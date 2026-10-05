package main

import (
	"encoding/json"
	"fmt"
	"net/http"
	"time"

	"github.com/go-chi/chi/v5"
)

// decodeBody strictly decodes a size-limited JSON request body.
func decodeBody(w http.ResponseWriter, r *http.Request, v interface{}) error {
	r.Body = http.MaxBytesReader(w, r.Body, maxRequestBody)
	dec := json.NewDecoder(r.Body)
	dec.DisallowUnknownFields()
	return dec.Decode(v)
}

// createExecutionHandler accepts an inspection request identical to
// POST /inspect, runs the joint solver and, only on success, freezes
// map, tasks, assignment and routes into a new execution.
func (s *execStore) createExecutionHandler(w http.ResponseWriter, r *http.Request) {
	var req InspectRequest
	if err := decodeBody(w, r, &req); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "invalid JSON: " + err.Error()})
		return
	}
	grid, err := validateInspectRequest(&req)
	if err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": err.Error()})
		return
	}
	assign, paths, arrivals, dwellStarts, cost, status := solveInspect(
		r.Context(), grid, req.Vehicles, req.Tasks, req.MaxSteps, req.NodeBudget)
	if status != StatusSuccess {
		// No execution is created without a proven optimal plan.
		writeJSON(w, http.StatusOK, map[string]string{"status": status})
		return
	}

	plan := InspectResponse{Status: status, TotalCost: cost}
	plan.Assignments = make([]InspectAssignment, len(req.Vehicles))
	for i, v := range req.Vehicles {
		task := req.Tasks[assign[i]]
		plan.Assignments[i] = InspectAssignment{
			VehicleID:   v.ID,
			TaskID:      task.ID,
			JobStart:    dwellStarts[i],
			JobFinish:   dwellStarts[i] + task.Dwell,
			Path:        paths[i],
			ParkingTime: arrivals[i],
		}
	}

	s.mu.Lock()
	defer s.mu.Unlock()
	if len(s.execs) >= s.maxExecs {
		writeJSON(w, http.StatusTooManyRequests, map[string]string{"error": "execution limit reached"})
		return
	}
	e := &Execution{
		ID:        s.newID(),
		CreatedAt: time.Now().UTC().Format(time.RFC3339),
		Request:   req,
		Plan:      plan,
		obs:       map[string]map[int]Cell{},
	}
	if err := s.persist(e); err != nil {
		writeJSON(w, http.StatusInternalServerError, map[string]string{"error": "persist: " + err.Error()})
		return
	}
	s.execs[e.ID] = e
	writeJSON(w, http.StatusCreated, map[string]interface{}{
		"status":       status,
		"execution_id": e.ID,
		"plan":         plan,
	})
}

// snapshot renders the consistent view returned by both the query and
// the seal endpoints.
func (e *Execution) snapshot() map[string]interface{} {
	report := e.Report
	if report == nil {
		report = e.verify()
	}
	return map[string]interface{}{
		"execution_id": e.ID,
		"created_at":   e.CreatedAt,
		"sealed":       e.Sealed,
		"request":      e.Request,
		"plan":         e.Plan,
		"observations": e.observationList(),
		"report":       report,
	}
}

func (s *execStore) getExecutionHandler(w http.ResponseWriter, r *http.Request) {
	s.mu.Lock()
	defer s.mu.Unlock()
	e := s.execs[chi.URLParam(r, "id")]
	if e == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "unknown execution"})
		return
	}
	writeJSON(w, http.StatusOK, e.snapshot())
}

// addObservationsHandler ingests one batch. The batch is validated as
// a unit: any unknown vehicle, out-of-range time, out-of-bounds
// coordinate or conflicting duplicate rejects the whole batch and
// stores nothing. Observations on obstacles or with illegal moves are
// kept as evidence for the verification report.
func (s *execStore) addObservationsHandler(w http.ResponseWriter, r *http.Request) {
	var batch ObservationBatch
	if err := decodeBody(w, r, &batch); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "invalid JSON: " + err.Error()})
		return
	}
	if len(batch.Observations) == 0 {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "observations must not be empty"})
		return
	}

	s.mu.Lock()
	defer s.mu.Unlock()
	e := s.execs[chi.URLParam(r, "id")]
	if e == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "unknown execution"})
		return
	}
	if e.Sealed {
		writeJSON(w, http.StatusConflict, map[string]string{"error": "execution is sealed"})
		return
	}
	if len(batch.Observations) > s.maxBatch {
		writeJSON(w, http.StatusBadRequest, map[string]string{
			"error": fmt.Sprintf("batch exceeds limit of %d observations", s.maxBatch)})
		return
	}

	known := map[string]bool{}
	for _, v := range e.Request.Vehicles {
		known[v.ID] = true
	}
	seen := map[string]map[int]Cell{}
	for _, o := range batch.Observations {
		if !known[o.VehicleID] {
			writeJSON(w, http.StatusBadRequest, map[string]string{
				"error": fmt.Sprintf("unknown vehicle %q", o.VehicleID)})
			return
		}
		if o.Time < 0 || o.Time > e.Request.MaxSteps {
			writeJSON(w, http.StatusBadRequest, map[string]string{
				"error": fmt.Sprintf("time %d out of range 0..%d", o.Time, e.Request.MaxSteps)})
			return
		}
		if o.Cell.X < 0 || o.Cell.X >= e.Request.Width || o.Cell.Y < 0 || o.Cell.Y >= e.Request.Height {
			writeJSON(w, http.StatusBadRequest, map[string]string{
				"error": fmt.Sprintf("coordinate (%d,%d) out of bounds", o.Cell.X, o.Cell.Y)})
			return
		}
		if prev, ok := seen[o.VehicleID][o.Time]; ok && prev != o.Cell {
			writeJSON(w, http.StatusConflict, map[string]string{
				"error": fmt.Sprintf("conflicting positions for vehicle %q at time %d", o.VehicleID, o.Time)})
			return
		}
		if seen[o.VehicleID] == nil {
			seen[o.VehicleID] = map[int]Cell{}
		}
		seen[o.VehicleID][o.Time] = o.Cell
		if prev, ok := e.obs[o.VehicleID][o.Time]; ok && prev != o.Cell {
			writeJSON(w, http.StatusConflict, map[string]string{
				"error": fmt.Sprintf("conflicting positions for vehicle %q at time %d", o.VehicleID, o.Time)})
			return
		}
	}

	// Merge into a copy first: a persist failure must not leave
	// partially applied in-memory state.
	merged := map[string]map[int]Cell{}
	for vid, cells := range e.obs {
		merged[vid] = map[int]Cell{}
		for t, c := range cells {
			merged[vid][t] = c
		}
	}
	added, duplicates := 0, 0
	for _, o := range batch.Observations {
		if merged[o.VehicleID] == nil {
			merged[o.VehicleID] = map[int]Cell{}
		}
		if _, ok := merged[o.VehicleID][o.Time]; ok {
			duplicates++
			continue
		}
		merged[o.VehicleID][o.Time] = o.Cell
		added++
	}
	updated := *e
	updated.obs = merged
	if err := s.persist(&updated); err != nil {
		writeJSON(w, http.StatusInternalServerError, map[string]string{"error": "persist: " + err.Error()})
		return
	}
	e.obs = merged
	writeJSON(w, http.StatusOK, map[string]interface{}{
		"status":     "accepted",
		"added":      added,
		"duplicates": duplicates,
	})
}

// sealExecutionHandler freezes the execution. The computed report is
// persisted with the record; further observation uploads are rejected
// and repeated seals return the original frozen report.
func (s *execStore) sealExecutionHandler(w http.ResponseWriter, r *http.Request) {
	s.mu.Lock()
	defer s.mu.Unlock()
	e := s.execs[chi.URLParam(r, "id")]
	if e == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "unknown execution"})
		return
	}
	if e.Sealed {
		writeJSON(w, http.StatusOK, e.snapshot())
		return
	}
	updated := *e
	updated.Sealed = true
	updated.Report = e.verify()
	updated.Report.Sealed = true
	if err := s.persist(&updated); err != nil {
		writeJSON(w, http.StatusInternalServerError, map[string]string{"error": "persist: " + err.Error()})
		return
	}
	e.Sealed = true
	e.Report = updated.Report
	writeJSON(w, http.StatusOK, e.snapshot())
}
