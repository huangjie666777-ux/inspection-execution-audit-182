package main

import (
	cryptorand "crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"sync"
	"time"
)

var errNotFound = errors.New("execution not found")
var errConflict = errors.New("observation conflict")

type Store struct {
	mu       sync.Mutex
	dir      string
	maxItems int
	records  map[string]*ExecutionRecord
}

func newExecutionStore(dir string, maxItems int) (*Store, error) {
	s := &Store{dir: dir, maxItems: maxItems, records: map[string]*ExecutionRecord{}}
	if dir != "" {
		if err := os.MkdirAll(dir, 0o755); err != nil {
			return nil, err
		}
		entries, err := os.ReadDir(dir)
		if err != nil {
			return nil, err
		}
		for _, entry := range entries {
			if entry.IsDir() || filepath.Ext(entry.Name()) != ".json" {
				continue
			}
			raw, err := os.ReadFile(filepath.Join(dir, entry.Name()))
			if err != nil {
				return nil, err
			}
			var rec ExecutionRecord
			if err := json.Unmarshal(raw, &rec); err != nil {
				return nil, fmt.Errorf("load %s: %w", entry.Name(), err)
			}
			s.records[rec.ID] = &rec
		}
	}
	return s, nil
}

func executionID() (string, error) {
	var b [16]byte
	if _, err := cryptorand.Read(b[:]); err != nil {
		return "", err
	}
	return "exec_" + hex.EncodeToString(b[:]), nil
}

func cloneRecord(rec *ExecutionRecord) *ExecutionRecord {
	raw, _ := json.Marshal(rec)
	var out ExecutionRecord
	_ = json.Unmarshal(raw, &out)
	return &out
}

func (s *Store) persistLocked(rec *ExecutionRecord) error {
	if s.dir == "" {
		return nil
	}
	raw, err := json.MarshalIndent(rec, "", "  ")
	if err != nil {
		return err
	}
	final := filepath.Join(s.dir, rec.ID+".json")
	tmp := final + ".tmp"
	if err := os.WriteFile(tmp, raw, 0o600); err != nil {
		_ = os.Remove(tmp)
		return err
	}
	if err := os.Rename(tmp, final); err != nil {
		_ = os.Remove(tmp)
		return err
	}
	return nil
}

func (s *Store) Create(rec *ExecutionRecord) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if len(s.records) >= s.maxItems {
		return errExecutionLimit
	}
	if _, exists := s.records[rec.ID]; exists {
		return fmt.Errorf("execution id collision")
	}
	if err := s.persistLocked(rec); err != nil {
		return err
	}
	s.records[rec.ID] = rec
	return nil
}

func (s *Store) Get(id string) (*ExecutionRecord, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	rec, ok := s.records[id]
	if !ok {
		return nil, errNotFound
	}
	return cloneRecord(rec), nil
}

func validateBatchLocked(rec *ExecutionRecord, batch []Observation) error {
	if len(batch) == 0 {
		return errors.New("observations must not be empty")
	}
	known := map[string]bool{}
	for _, pv := range rec.Plan.Vehicles {
		known[pv.VehicleID] = true
	}
	seen := map[string]Observation{}
	for _, ob := range batch {
		if !known[ob.VehicleID] {
			return fmt.Errorf("unknown vehicle %q", ob.VehicleID)
		}
		if ob.Time < 0 || ob.Time > rec.Plan.MaxSteps {
			return fmt.Errorf("vehicle %q time %d outside 0..%d", ob.VehicleID, ob.Time, rec.Plan.MaxSteps)
		}
		if !inBounds(rec.Plan, ob.Cell) {
			return fmt.Errorf("vehicle %q coordinate (%d,%d) outside map", ob.VehicleID, ob.Cell.X, ob.Cell.Y)
		}
		key := fmt.Sprintf("%s@%d", ob.VehicleID, ob.Time)
		if prior, ok := seen[key]; ok && prior.Cell != ob.Cell {
			return fmt.Errorf("%w: vehicle %q reports different cells at time %d", errConflict, ob.VehicleID, ob.Time)
		}
		seen[key] = ob
	}
	return nil
}

func (s *Store) AddObservations(id string, batch []Observation) (*ExecutionRecord, int, int, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	rec, ok := s.records[id]
	if !ok {
		return nil, 0, 0, errNotFound
	}
	if rec.Sealed {
		return nil, 0, 0, errSealed
	}
	if err := validateBatchLocked(rec, batch); err != nil {
		return nil, 0, 0, err
	}
	candidate := cloneRecord(rec)
	accepted, duplicate := 0, 0
	seenBatch := map[string]bool{}
	for _, ob := range batch {
		key := fmt.Sprintf("%s@%d", ob.VehicleID, ob.Time)
		if seenBatch[key] {
			duplicate++
			continue
		}
		seenBatch[key] = true
		existing := candidate.Observations[ob.VehicleID]
		found := false
		for _, p := range existing {
			if p.Time == ob.Time {
				if p.Cell != ob.Cell {
					return nil, 0, 0, fmt.Errorf("%w: vehicle %q reports different cells at time %d", errConflict, ob.VehicleID, ob.Time)
				}
				found = true
				break
			}
		}
		if found {
			duplicate++
			continue
		}
		existing = append(existing, ObservedPoint{Time: ob.Time, Cell: ob.Cell})
		sort.Slice(existing, func(i, j int) bool { return existing[i].Time < existing[j].Time })
		candidate.Observations[ob.VehicleID] = existing
		accepted++
	}
	if err := s.persistLocked(candidate); err != nil {
		return nil, 0, 0, err
	}
	s.records[id] = candidate
	return cloneRecord(candidate), accepted, duplicate, nil
}

func (s *Store) Seal(id string) (*ExecutionRecord, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	rec, ok := s.records[id]
	if !ok {
		return nil, errNotFound
	}
	if rec.Sealed {
		return cloneRecord(rec), nil
	}
	candidate := cloneRecord(rec)
	now := time.Now().UTC()
	report := buildReport(candidate)
	candidate.Sealed = true
	candidate.SealedAt = &now
	candidate.Report = &report
	if err := s.persistLocked(candidate); err != nil {
		return nil, err
	}
	s.records[id] = candidate
	return cloneRecord(candidate), nil
}

func snapshot(rec *ExecutionRecord) ExecutionSnapshot {
	report := rec.Report
	if report == nil {
		r := buildReport(rec)
		report = &r
	}
	return ExecutionSnapshot{
		ID: rec.ID, CreatedAt: rec.CreatedAt, Sealed: rec.Sealed, SealedAt: rec.SealedAt,
		Plan: rec.Plan, Observations: cloneRecord(rec).Observations, Report: *report,
	}
}

var (
	errSealed         = errors.New("execution is sealed")
	errExecutionLimit = errors.New("execution limit reached")
)
