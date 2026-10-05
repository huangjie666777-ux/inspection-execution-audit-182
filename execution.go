package main

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"time"
)

// Observation is one reported position sample of a vehicle at an
// integer time. Observations may arrive in any order and in batches.
type Observation struct {
	VehicleID string `json:"vehicle_id"`
	Time      int    `json:"time"`
	Cell      Cell   `json:"cell"`
}

// ObservationBatch is the JSON body accepted by
// POST /executions/{id}/observations.
type ObservationBatch struct {
	Observations []Observation `json:"observations"`
}

// Execution is one accepted inspection plan under execution
// verification. The map, tasks, assignment and routes are frozen at
// creation; only observations and the seal flag change afterwards.
type Execution struct {
	ID        string
	CreatedAt string
	Sealed    bool
	Request   InspectRequest
	Plan      InspectResponse
	obs       map[string]map[int]Cell // vehicle -> time -> cell
	Report    *VerifyReport           // frozen at seal time
}

// plannedPos returns the planned cell of vehicle vid at time t. The
// planned route covers steps 0..parking; after parking the vehicle
// stays at its goal, so the route is extended by the final cell up to
// max_steps.
func (e *Execution) plannedPos(vid string, t int) Cell {
	for _, a := range e.Plan.Assignments {
		if a.VehicleID == vid {
			if t < len(a.Path) {
				return a.Path[t]
			}
			return a.Path[len(a.Path)-1]
		}
	}
	return Cell{}
}

// observationList flattens the observation map in a deterministic
// (vehicle, time) order for persistence and snapshots.
func (e *Execution) observationList() []Observation {
	out := make([]Observation, 0)
	for vid, cells := range e.obs {
		for t, c := range cells {
			out = append(out, Observation{VehicleID: vid, Time: t, Cell: c})
		}
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i].VehicleID != out[j].VehicleID {
			return out[i].VehicleID < out[j].VehicleID
		}
		return out[i].Time < out[j].Time
	})
	return out
}

// persistedExecution is the on-disk JSON form of an Execution.
type persistedExecution struct {
	ID           string          `json:"id"`
	CreatedAt    string          `json:"created_at"`
	Sealed       bool            `json:"sealed"`
	Request      InspectRequest  `json:"request"`
	Plan         InspectResponse `json:"plan"`
	Observations []Observation   `json:"observations"`
	Report       *VerifyReport   `json:"report,omitempty"`
}

func (e *Execution) toPersisted() *persistedExecution {
	return &persistedExecution{
		ID:           e.ID,
		CreatedAt:    e.CreatedAt,
		Sealed:       e.Sealed,
		Request:      e.Request,
		Plan:         e.Plan,
		Observations: e.observationList(),
		Report:       e.Report,
	}
}

func (p *persistedExecution) toExecution() *Execution {
	e := &Execution{
		ID:        p.ID,
		CreatedAt: p.CreatedAt,
		Sealed:    p.Sealed,
		Request:   p.Request,
		Plan:      p.Plan,
		obs:       map[string]map[int]Cell{},
		Report:    p.Report,
	}
	for _, o := range p.Observations {
		if e.obs[o.VehicleID] == nil {
			e.obs[o.VehicleID] = map[int]Cell{}
		}
		e.obs[o.VehicleID][o.Time] = o.Cell
	}
	return e
}

const (
	defaultMaxExecutions = 128
	defaultMaxBatch      = 2048
	maxRequestBody       = 1 << 20 // 1 MiB
)

// execStore owns all executions. A single mutex serializes creation,
// observation ingest and sealing so concurrent requests commit in a
// well-defined order. When dir is empty the store is memory-only.
type execStore struct {
	mu       sync.Mutex
	dir      string
	maxExecs int
	maxBatch int
	execs    map[string]*Execution
	seq      int
}

func newExecStore(dir string, maxExecs, maxBatch int) *execStore {
	if maxExecs <= 0 {
		maxExecs = defaultMaxExecutions
	}
	if maxBatch <= 0 {
		maxBatch = defaultMaxBatch
	}
	return &execStore{dir: dir, maxExecs: maxExecs, maxBatch: maxBatch, execs: map[string]*Execution{}}
}

// load reads every persisted execution from the store directory.
func (s *execStore) load() error {
	if s.dir == "" {
		return nil
	}
	if err := os.MkdirAll(s.dir, 0o755); err != nil {
		return err
	}
	entries, err := os.ReadDir(s.dir)
	if err != nil {
		return err
	}
	for _, entry := range entries {
		if entry.IsDir() || !strings.HasSuffix(entry.Name(), ".json") {
			continue
		}
		data, err := os.ReadFile(filepath.Join(s.dir, entry.Name()))
		if err != nil {
			return fmt.Errorf("load %s: %w", entry.Name(), err)
		}
		var p persistedExecution
		if err := json.Unmarshal(data, &p); err != nil {
			return fmt.Errorf("parse %s: %w", entry.Name(), err)
		}
		e := p.toExecution()
		s.execs[e.ID] = e
	}
	return nil
}

// persist atomically writes the execution record: temp file, fsync,
// rename. A failure leaves the previous on-disk state untouched.
func (s *execStore) persist(e *Execution) error {
	if s.dir == "" {
		return nil
	}
	data, err := json.MarshalIndent(e.toPersisted(), "", "  ")
	if err != nil {
		return err
	}
	final := filepath.Join(s.dir, e.ID+".json")
	tmp := final + ".tmp"
	f, err := os.OpenFile(tmp, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0o644)
	if err != nil {
		return err
	}
	if _, err := f.Write(data); err != nil {
		_ = f.Close()
		_ = os.Remove(tmp)
		return err
	}
	if err := f.Sync(); err != nil {
		_ = f.Close()
		_ = os.Remove(tmp)
		return err
	}
	if err := f.Close(); err != nil {
		_ = os.Remove(tmp)
		return err
	}
	return os.Rename(tmp, final)
}

// newID returns a unique, collision-safe execution ID.
func (s *execStore) newID() string {
	for {
		s.seq++
		id := fmt.Sprintf("exec-%d-%d", time.Now().UnixNano(), s.seq)
		if s.execs[id] == nil {
			return id
		}
	}
}
