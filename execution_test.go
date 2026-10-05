package main

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"
)

func testApp(t *testing.T, dir string) (*App, http.Handler) {
	t.Helper()
	app, err := newApp(dir, 10, 4)
	if err != nil {
		t.Fatalf("newApp: %v", err)
	}
	return app, newAppRouter(app)
}

func doJSON(t *testing.T, h http.Handler, method, path, body string) (int, map[string]interface{}) {
	t.Helper()
	var r *http.Request
	if body == "" {
		r = httptest.NewRequest(method, path, nil)
	} else {
		r = httptest.NewRequest(method, path, bytes.NewBufferString(body))
	}
	w := httptest.NewRecorder()
	h.ServeHTTP(w, r)
	var out map[string]interface{}
	_ = json.Unmarshal(w.Body.Bytes(), &out)
	return w.Code, out
}

func createExecutionFromExample(t *testing.T, h http.Handler) string {
	t.Helper()
	raw, err := os.ReadFile("examples/inspect.json")
	if err != nil {
		t.Fatal(err)
	}
	code, out := doJSON(t, h, http.MethodPost, "/inspect/executions", string(raw))
	if code != http.StatusCreated || out["execution_id"] == nil {
		t.Fatalf("code=%d out=%v", code, out)
	}
	return out["execution_id"].(string)
}

func ob(vehicleID string, tm, x, y int) Observation {
	return Observation{VehicleID: vehicleID, Time: tm, Cell: Cell{X: x, Y: y}}
}

func plannedObservations(t *testing.T, out map[string]interface{}) []Observation {
	t.Helper()
	maxSteps := int(out["plan"].(map[string]interface{})["max_steps"].(float64))
	vehicles := out["plan"].(map[string]interface{})["vehicles"].([]interface{})
	obs := []Observation{}
	for _, item := range vehicles {
		v := item.(map[string]interface{})
		vid := v["vehicle_id"].(string)
		parking := int(v["parking_time"].(float64))
		path := v["path"].([]interface{})
		for tm := 0; tm <= maxSteps; tm++ {
			idx := tm
			if idx > parking {
				idx = parking
			}
			cell := path[idx].(map[string]interface{})
			obs = append(obs, ob(vid, tm, int(cell["x"].(float64)), int(cell["y"].(float64))))
		}
	}
	return obs
}

func marshalObservations(t *testing.T, obs []Observation) string {
	t.Helper()
	raw, err := json.Marshal(ObservationBatch{Observations: obs})
	if err != nil {
		t.Fatal(err)
	}
	return string(raw)
}

func TestExecutionFullPlanPassesAndSeals(t *testing.T) {
	_, h := testApp(t, "")
	raw, err := os.ReadFile("examples/inspect.json")
	if err != nil {
		t.Fatal(err)
	}
	_, created := doJSON(t, h, http.MethodPost, "/inspect/executions", string(raw))
	id := created["execution_id"].(string)
	obs := plannedObservations(t, created)
	for i, j := 0, len(obs)-1; i < j; i, j = i+1, j-1 {
		obs[i], obs[j] = obs[j], obs[i]
	}
	body := marshalObservations(t, obs)
	code, report := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", body)
	if code != http.StatusOK || report["report"].(map[string]interface{})["status"] != "passed" {
		t.Fatalf("code=%d report=%v", code, report)
	}
	code, sealed := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/seal", "")
	if code != http.StatusOK || sealed["report"].(map[string]interface{})["status"] != "passed" || sealed["sealed"] != true {
		t.Fatalf("code=%d sealed=%v", code, sealed)
	}
	code, again := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/seal", "")
	if code != http.StatusOK || again["report"].(map[string]interface{})["status"] != "passed" {
		t.Fatalf("repeat seal code=%d %v", code, again)
	}
	code, _ = doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", body)
	if code != http.StatusConflict {
		t.Fatalf("post after seal code=%d", code)
	}
}

func TestExecutionBatchAtomicIdempotenceAndEvidence(t *testing.T) {
	_, h := testApp(t, "")
	id := createExecutionFromExample(t, h)
	good := []Observation{ob("a", 0, 0, 0)}
	if code, out := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", marshalObservations(t, good)); code != 200 || out["report"].(map[string]interface{})["status"] != "unknown" {
		t.Fatalf("good code=%d out=%v", code, out)
	}
	batch := append(good, ob("a", 2, 0, 0), ob("ghost", 0, 0, 0))
	if code, _ := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", marshalObservations(t, batch)); code != http.StatusBadRequest {
		t.Fatalf("unknown vehicle code=%d", code)
	}
	batch = append(good, ob("a", 1, 9, 0))
	if code, _ := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", marshalObservations(t, batch)); code != http.StatusBadRequest {
		t.Fatalf("out of bounds code=%d", code)
	}
	if code, out := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", marshalObservations(t, append(good, good[0]))); code != 200 || out["duplicate"].(float64) != 2 {
		t.Fatalf("idempotent code=%d out=%v", code, out)
	}
	conflict := append(good, ob("a", 0, 1, 0))
	if code, _ := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", marshalObservations(t, conflict)); code != http.StatusConflict {
		t.Fatalf("conflict code=%d", code)
	}
	bad := []Observation{ob("b", 0, 1, 1), ob("b", 1, 1, 0), ob("b", 2, 0, 0)}
	code, out := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", marshalObservations(t, bad))
	if code != 200 || out["report"].(map[string]interface{})["status"] != "failed" {
		t.Fatalf("evidence report code=%d out=%v", code, out)
	}
}

func TestExecutionMissingObservationsAreUnknown(t *testing.T) {
	_, h := testApp(t, "")
	id := createExecutionFromExample(t, h)
	obs := []Observation{ob("a", 0, 0, 0)}
	code, out := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", marshalObservations(t, obs))
	if code != http.StatusOK || out["report"].(map[string]interface{})["status"] != "unknown" {
		t.Fatalf("code=%d out=%v", code, out)
	}
	tasks := out["report"].(map[string]interface{})["task_results"].([]interface{})
	for _, item := range tasks {
		if item.(map[string]interface{})["status"] != "unknown" {
			t.Fatalf("partial task result = %v", item)
		}
	}
}

func TestVerifierDetectsSwapObstacleAndIncompleteTasks(t *testing.T) {
	rec := &ExecutionRecord{
		Plan: ExecutionPlan{
			MaxSteps: 1, Width: 3, Height: 1, Obstacles: []Cell{{X: 2, Y: 0}},
			Tasks: []TaskSpec{
				{ID: "t1", Cell: Cell{0, 0}, EarliestStart: 0, LatestFinish: 1, Dwell: 1},
				{ID: "t2", Cell: Cell{1, 0}, EarliestStart: 0, LatestFinish: 1, Dwell: 1},
			},
			Vehicles: []PlannedVehicle{
				{VehicleID: "a", TaskID: "t1", JobStart: 0, JobFinish: 1, ParkingTime: 1,
					Path: []Cell{{0, 0}, {0, 0}}},
				{VehicleID: "b", TaskID: "t2", JobStart: 0, JobFinish: 1, ParkingTime: 1,
					Path: []Cell{{1, 0}, {1, 0}}},
			},
		},
		Observations: map[string][]ObservedPoint{
			"a": {{Time: 0, Cell: Cell{2, 0}}, {Time: 1, Cell: Cell{1, 0}}},
			"b": {{Time: 0, Cell: Cell{2, 0}}, {Time: 1, Cell: Cell{0, 0}}},
		},
	}
	report := buildReport(rec)
	if report.Status != ReportFailed {
		t.Fatalf("status=%s report=%+v", report.Status, report)
	}
	types := map[string]bool{}
	for _, ev := range report.Evidence {
		types[ev.Type] = true
	}
	for _, want := range []string{"obstacle", "vertex_conflict"} {
		if !types[want] {
			t.Fatalf("missing evidence %s in %+v", want, report.Evidence)
		}
	}
	for _, task := range report.TaskResults {
		if task.Status != TaskIncomplete {
			t.Fatalf("task %s status=%s", task.TaskID, task.Status)
		}
	}

	swap := &ExecutionRecord{Plan: rec.Plan, Observations: map[string][]ObservedPoint{
		"a": {{Time: 0, Cell: Cell{0, 0}}, {Time: 1, Cell: Cell{1, 0}}},
		"b": {{Time: 0, Cell: Cell{1, 0}}, {Time: 1, Cell: Cell{0, 0}}},
	}}
	swapReport := buildReport(swap)
	foundSwap := false
	for _, ev := range swapReport.Evidence {
		foundSwap = foundSwap || ev.Type == "edge_swap"
	}
	if !foundSwap {
		t.Fatalf("missing edge swap in %+v", swapReport.Evidence)
	}
}

func TestExecutionPersistenceRestoresSealedSnapshot(t *testing.T) {
	dir := t.TempDir()
	app, h := testApp(t, dir)
	id := createExecutionFromExample(t, h)
	obs := []Observation{ob("a", 0, 0, 1)}
	if code, _ := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/observations", marshalObservations(t, obs)); code != 200 {
		t.Fatalf("observation code=%d", code)
	}
	if code, sealed := doJSON(t, h, http.MethodPost, "/inspect/executions/"+id+"/seal", ""); code != 200 || sealed["report"].(map[string]interface{})["status"] != "failed" {
		t.Fatalf("seal=%d %v", code, sealed)
	}
	if _, err := os.Stat(filepath.Join(dir, id+".json")); err != nil {
		t.Fatalf("record file: %v", err)
	}
	restored, err := newApp(dir, 10, 4)
	if err != nil {
		t.Fatal(err)
	}
	rec, err := restored.store.Get(id)
	if err != nil {
		t.Fatal(err)
	}
	if !rec.Sealed || rec.Report == nil || rec.Report.Status != "failed" {
		t.Fatalf("restored record=%+v", rec)
	}
	if _, _, _, err := restored.store.AddObservations(id, []Observation{{VehicleID: "a", Time: 1, Cell: Cell{0, 0}}}); err != errSealed {
		t.Fatalf("restored seal err=%v", err)
	}
	app.store.mu.Lock()
	app.store.mu.Unlock()
}
