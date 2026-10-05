package main

import (
	"encoding/json"
	"errors"
	"net/http"
	"os"
	"strconv"
	"time"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
)

type App struct {
	store         *Store
	plannerTokens chan struct{}
	maxBodyBytes  int
}

const (
	defaultMaxExecutions = 1000
	defaultMaxInFlight   = 8
	defaultMaxBodyBytes  = 1 << 20
)

func envInt(name string, fallback int) int {
	raw := os.Getenv(name)
	if raw == "" {
		return fallback
	}
	v, err := strconv.Atoi(raw)
	if err != nil || v < 1 {
		return fallback
	}
	return v
}

func newApp(dataDir string, maxExecutions, maxInFlight int) (*App, error) {
	store, err := newExecutionStore(dataDir, maxExecutions)
	if err != nil {
		return nil, err
	}
	return &App{store: store, plannerTokens: make(chan struct{}, maxInFlight),
		maxBodyBytes: envInt("MAX_REQUEST_BODY_BYTES", defaultMaxBodyBytes)}, nil
}

// newRouter wires the HTTP API. Each request gets an independent planner
// run: constraints and node budget live entirely inside solveCBS.
func newRouter() http.Handler {
	app, _ := newApp("", defaultMaxExecutions, defaultMaxInFlight)
	return newAppRouter(app)
}

func newAppRouter(app *App) http.Handler {
	r := chi.NewRouter()
	r.Use(middleware.Recoverer)
	r.Use(func(next http.Handler) http.Handler {
		return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			r.Body = http.MaxBytesReader(w, r.Body, int64(app.maxBodyBytes))
			next.ServeHTTP(w, r)
		})
	})
	r.Post("/plan", app.limitPlanner(planHandler))
	r.Post("/inspect", app.limitPlanner(inspectHandler))
	r.Post("/inspect/executions", app.createExecutionHandler)
	r.Post("/inspect/executions/{executionID}/observations", app.observationHandler)
	r.Get("/inspect/executions/{executionID}", app.getExecutionHandler)
	r.Post("/inspect/executions/{executionID}/seal", app.sealExecutionHandler)
	r.Get("/healthz", func(w http.ResponseWriter, _ *http.Request) {
		w.WriteHeader(http.StatusOK)
		_, _ = w.Write([]byte("ok"))
	})
	return r
}

func (app *App) limitPlanner(next http.HandlerFunc) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		select {
		case app.plannerTokens <- struct{}{}:
			defer func() { <-app.plannerTokens }()
			next(w, r)
		case <-r.Context().Done():
			writeJSON(w, http.StatusServiceUnavailable, map[string]string{"error": "planner capacity exhausted"})
		}
	}
}

func decodeInspectRequest(w http.ResponseWriter, r *http.Request) (*InspectRequest, bool) {
	var req InspectRequest
	dec := json.NewDecoder(r.Body)
	dec.DisallowUnknownFields()
	if err := dec.Decode(&req); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "invalid JSON: " + err.Error()})
		return nil, false
	}
	return &req, true
}

func writeJSON(w http.ResponseWriter, code int, v interface{}) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(code)
	_ = json.NewEncoder(w).Encode(v)
}

func planHandler(w http.ResponseWriter, r *http.Request) {
	var req PlanRequest
	dec := json.NewDecoder(r.Body)
	dec.DisallowUnknownFields()
	if err := dec.Decode(&req); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "invalid JSON: " + err.Error()})
		return
	}
	grid, err := validateRequest(&req)
	if err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": err.Error()})
		return
	}

	// r.Context() carries client cancellation: an aborted request stops
	// its CBS search without affecting concurrent requests.
	paths, arrivals, cost, status := solveCBS(r.Context(), grid, req.Vehicles, req.MaxSteps, req.NodeBudget)

	resp := PlanResponse{Status: status}
	if status == StatusSuccess {
		resp.TotalCost = cost
		resp.Vehicles = make([]VehiclePath, len(req.Vehicles))
		for i, v := range req.Vehicles {
			resp.Vehicles[i] = VehiclePath{ID: v.ID, Path: paths[i], ArrivalTime: arrivals[i]}
		}
	}
	writeJSON(w, http.StatusOK, resp)
}

func inspectHandler(w http.ResponseWriter, r *http.Request) {
	req, ok := decodeInspectRequest(w, r)
	if !ok {
		return
	}
	grid, err := validateInspectRequest(req)
	if err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": err.Error()})
		return
	}

	// r.Context() carries client cancellation: an aborted request stops
	// its joint search without affecting concurrent requests.
	assign, paths, arrivals, dwellStarts, cost, status := solveInspect(
		r.Context(), grid, req.Vehicles, req.Tasks, req.MaxSteps, req.NodeBudget)

	resp := InspectResponse{Status: status}
	if status == StatusSuccess {
		resp.TotalCost = cost
		resp.Assignments = make([]InspectAssignment, len(req.Vehicles))
		for i, v := range req.Vehicles {
			task := req.Tasks[assign[i]]
			resp.Assignments[i] = InspectAssignment{
				VehicleID:   v.ID,
				TaskID:      task.ID,
				JobStart:    dwellStarts[i],
				JobFinish:   dwellStarts[i] + task.Dwell,
				Path:        paths[i],
				ParkingTime: arrivals[i],
			}
		}
	}
	writeJSON(w, http.StatusOK, resp)
}

func (app *App) createExecutionHandler(w http.ResponseWriter, r *http.Request) {
	app.limitPlanner(func(w http.ResponseWriter, r *http.Request) {
		req, ok := decodeInspectRequest(w, r)
		if !ok {
			return
		}
		grid, err := validateInspectRequest(req)
		if err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]string{"error": err.Error()})
			return
		}
		assign, paths, arrivals, dwellStarts, cost, status := solveInspect(
			r.Context(), grid, req.Vehicles, req.Tasks, req.MaxSteps, req.NodeBudget)
		if status != StatusSuccess {
			writeJSON(w, http.StatusOK, CreateExecutionResponse{Status: status})
			return
		}
		id, err := executionID()
		if err != nil {
			writeJSON(w, http.StatusInternalServerError, map[string]string{"error": err.Error()})
			return
		}
		rec := &ExecutionRecord{
			ID: id, CreatedAt: time.Now().UTC(),
			Request: *req, Observations: map[string][]ObservedPoint{},
			Plan: ExecutionPlan{Status: status, TotalCost: cost, MaxSteps: req.MaxSteps,
				Width: req.Width, Height: req.Height, Obstacles: append([]Cell(nil), req.Obstacles...),
				Tasks: append([]TaskSpec(nil), req.Tasks...), Vehicles: make([]PlannedVehicle, len(req.Vehicles))},
		}
		for i, v := range req.Vehicles {
			task := req.Tasks[assign[i]]
			rec.Plan.Vehicles[i] = PlannedVehicle{
				VehicleID: v.ID, TaskID: task.ID, JobStart: dwellStarts[i],
				JobFinish: dwellStarts[i] + task.Dwell, ParkingTime: arrivals[i],
				Path: append([]Cell(nil), paths[i]...),
			}
		}
		if err := app.store.Create(rec); err != nil {
			if errors.Is(err, errExecutionLimit) {
				writeJSON(w, http.StatusTooManyRequests, map[string]string{"error": err.Error()})
				return
			}
			writeJSON(w, http.StatusInternalServerError, map[string]string{"error": err.Error()})
			return
		}
		writeJSON(w, http.StatusCreated, CreateExecutionResponse{ExecutionID: id, Status: status, Plan: &rec.Plan})
	})(w, r)
}

func (app *App) observationHandler(w http.ResponseWriter, r *http.Request) {
	var batch ObservationBatch
	dec := json.NewDecoder(r.Body)
	dec.DisallowUnknownFields()
	if err := dec.Decode(&batch); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "invalid JSON: " + err.Error()})
		return
	}
	rec, accepted, duplicate, err := app.store.AddObservations(chi.URLParam(r, "executionID"), batch.Observations)
	writeStoreError(w, err)
	if err != nil {
		return
	}
	writeJSON(w, http.StatusOK, ObservationResponse{
		ExecutionID: rec.ID, Accepted: accepted, Duplicate: duplicate, Report: buildReport(rec)})
}

func (app *App) getExecutionHandler(w http.ResponseWriter, r *http.Request) {
	rec, err := app.store.Get(chi.URLParam(r, "executionID"))
	writeStoreError(w, err)
	if err == nil {
		writeJSON(w, http.StatusOK, snapshot(rec))
	}
}

func (app *App) sealExecutionHandler(w http.ResponseWriter, r *http.Request) {
	rec, err := app.store.Seal(chi.URLParam(r, "executionID"))
	writeStoreError(w, err)
	if err == nil {
		writeJSON(w, http.StatusOK, snapshot(rec))
	}
}

func writeStoreError(w http.ResponseWriter, err error) {
	switch {
	case err == nil:
	case errors.Is(err, errNotFound):
		writeJSON(w, http.StatusNotFound, map[string]string{"error": err.Error()})
	case errors.Is(err, errSealed):
		writeJSON(w, http.StatusConflict, map[string]string{"error": err.Error()})
	case errors.Is(err, errConflict):
		writeJSON(w, http.StatusConflict, map[string]string{"error": err.Error()})
	default:
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": err.Error()})
	}
}
