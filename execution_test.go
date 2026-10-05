package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"net/http"
	"net/http/httptest"
	"testing"
)

const execCreateBody = `{
	"width": 6, "height": 2, "max_steps": 16, "node_budget": 20000,
	"vehicles": [
		{"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 5, "y": 0}},
		{"id": "b", "start": {"x": 1, "y": 1}, "goal": {"x": 0, "y": 1}}
	],
	"tasks": [
		{"id": "t1", "cell": {"x": 1, "y": 0}, "vehicles": ["a", "b"], "earliest_start": 0, "latest_finish": 12, "dwell": 1},
		{"id": "t2", "cell": {"x": 4, "y": 0}, "vehicles": ["a", "b"], "earliest_start": 0, "latest_finish": 12, "dwell": 1}
	]
}`

func doReq(t *testing.T, h http.Handler, method, path, body string) (int, map[string]interface{}) {
	t.Helper()
	req := httptest.NewRequest(method, path, bytes.NewBufferString(body))
	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, req)
	var out map[string]interface{}
	_ = json.Unmarshal(rec.Body.Bytes(), &out)
	return rec.Code, out
}

func createExec(t *testing.T, h http.Handler) string {
	t.Helper()
	code, out := doReq(t, h, http.MethodPost, "/executions", execCreateBody)
	if code != http.StatusCreated || out["execution_id"] == nil {
		t.Fatalf("create: code=%d out=%v", code, out)
	}
	return out["execution_id"].(string)
}

func TestCreateExecutionOnlyOnSuccess(t *testing.T) {
	h := newRouter()
	// no_solution: 1-wide corridor swap with tasks forces no plan.
	code, out := doReq(t, h, http.MethodPost, "/executions", `{
		"width": 2, "height": 1, "max_steps": 12, "node_budget": 20000,
		"vehicles": [
			{"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 1, "y": 0}},
			{"id": "b", "start": {"x": 1, "y": 0}, "goal": {"x": 0, "y": 0}}
		],
		"tasks": [
			{"id": "t1", "cell": {"x": 0, "y": 0}, "vehicles": ["a"], "earliest_start": 0, "latest_finish": 10, "dwell": 1},
			{"id": "t2", "cell": {"x": 1, "y": 0}, "vehicles": ["b"], "earliest_start": 0, "latest_finish": 10, "dwell": 1}
		]
	}`)
	if code != http.StatusOK || out["status"] == StatusSuccess || out["execution_id"] != nil {
		t.Fatalf("code=%d out=%v", code, out)
	}
}

func TestObservationBatchRules(t *testing.T) {
	h := newRouter()
	id := createExec(t, h)
	url := "/executions/" + id + "/observations"

	// Unknown vehicle rejects the whole batch.
	code, out := doReq(t, h, http.MethodPost, url, `{"observations": [
		{"vehicle_id": "a", "time": 0, "cell": {"x": 0, "y": 0}},
		{"vehicle_id": "zzz", "time": 0, "cell": {"x": 1, "y": 1}}]}`)
	if code != http.StatusBadRequest {
		t.Fatalf("unknown vehicle: code=%d out=%v", code, out)
	}
	// Nothing stored by the rejected batch.
	code, out = doReq(t, h, http.MethodGet, "/executions/"+id, "")
	if obs := out["observations"].([]interface{}); len(obs) != 0 {
		t.Fatalf("rejected batch stored observations: %v", obs)
	}

	// Out-of-range time.
	code, _ = doReq(t, h, http.MethodPost, url, `{"observations": [
		{"vehicle_id": "a", "time": 17, "cell": {"x": 0, "y": 0}}]}`)
	if code != http.StatusBadRequest {
		t.Fatalf("time range: code=%d", code)
	}
	// Out-of-bounds coordinate.
	code, _ = doReq(t, h, http.MethodPost, url, `{"observations": [
		{"vehicle_id": "a", "time": 0, "cell": {"x": 6, "y": 0}}]}`)
	if code != http.StatusBadRequest {
		t.Fatalf("bounds: code=%d", code)
	}
	// Conflicting duplicates inside one batch.
	code, _ = doReq(t, h, http.MethodPost, url, `{"observations": [
		{"vehicle_id": "a", "time": 0, "cell": {"x": 0, "y": 0}},
		{"vehicle_id": "a", "time": 0, "cell": {"x": 1, "y": 0}}]}`)
	if code != http.StatusConflict {
		t.Fatalf("intra-batch conflict: code=%d", code)
	}

	// Out-of-order accept; identical resend is idempotent.
	code, out = doReq(t, h, http.MethodPost, url, `{"observations": [
		{"vehicle_id": "a", "time": 2, "cell": {"x": 2, "y": 0}},
		{"vehicle_id": "a", "time": 0, "cell": {"x": 0, "y": 0}},
		{"vehicle_id": "a", "time": 1, "cell": {"x": 1, "y": 0}}]}`)
	if code != http.StatusOK || out["added"].(float64) != 3 {
		t.Fatalf("accept: code=%d out=%v", code, out)
	}
	code, out = doReq(t, h, http.MethodPost, url, `{"observations": [
		{"vehicle_id": "a", "time": 1, "cell": {"x": 1, "y": 0}}]}`)
	if code != http.StatusOK || out["added"].(float64) != 0 || out["duplicates"].(float64) != 1 {
		t.Fatalf("idempotent: code=%d out=%v", code, out)
	}
	// Same vehicle+time with a different position conflicts.
	code, _ = doReq(t, h, http.MethodPost, url, `{"observations": [
		{"vehicle_id": "a", "time": 1, "cell": {"x": 2, "y": 0}}]}`)
	if code != http.StatusConflict {
		t.Fatalf("stored conflict: code=%d", code)
	}
}

func TestSealFreezesExecution(t *testing.T) {
	h := newRouter()
	id := createExec(t, h)
	url := "/executions/" + id

	// Seal with no observations: everything missing -> unknown.
	code, out := doReq(t, h, http.MethodPost, url+"/seal", "")
	if code != http.StatusOK || out["sealed"] != true {
		t.Fatalf("seal: code=%d out=%v", code, out)
	}
	report := out["report"].(map[string]interface{})
	if report["status"] != VerifyUnknown {
		t.Fatalf("report = %v", report)
	}
	// Reporting after seal is rejected.
	code, _ = doReq(t, h, http.MethodPost, url+"/observations", `{"observations": [
		{"vehicle_id": "a", "time": 0, "cell": {"x": 0, "y": 0}}]}`)
	if code != http.StatusConflict {
		t.Fatalf("post-seal report: code=%d", code)
	}
	// Duplicate seal returns the original report.
	code, out2 := doReq(t, h, http.MethodPost, url+"/seal", "")
	if code != http.StatusOK {
		t.Fatalf("re-seal: code=%d", code)
	}
	r1, _ := json.Marshal(report)
	r2, _ := json.Marshal(out2["report"])
	if string(r1) != string(r2) {
		t.Fatalf("re-seal report differs: %s vs %s", r1, r2)
	}
	// Query returns the same frozen snapshot.
	_, out3 := doReq(t, h, http.MethodGet, url, "")
	r3, _ := json.Marshal(out3["report"])
	if string(r1) != string(r3) {
		t.Fatalf("query report differs: %s vs %s", r1, r3)
	}
}

func TestPersistenceAcrossRestart(t *testing.T) {
	dir := t.TempDir()
	store := newExecStore(dir, 0, 0)
	if err := store.load(); err != nil {
		t.Fatalf("load: %v", err)
	}
	h := newRouterWithStore(store)
	id := createExec(t, h)
	code, out := doReq(t, h, http.MethodPost, "/executions/"+id+"/observations", `{"observations": [
		{"vehicle_id": "a", "time": 0, "cell": {"x": 0, "y": 0}}]}`)
	if code != http.StatusOK {
		t.Fatalf("report: code=%d out=%v", code, out)
	}
	if _, so := doReq(t, h, http.MethodPost, "/executions/"+id+"/seal", ""); so["sealed"] != true {
		t.Fatalf("seal: %v", out)
	}

	// Simulate a restart: a fresh store over the same directory.
	store2 := newExecStore(dir, 0, 0)
	if err := store2.load(); err != nil {
		t.Fatalf("reload: %v", err)
	}
	h2 := newRouterWithStore(store2)
	code, out = doReq(t, h2, http.MethodGet, "/executions/"+id, "")
	if code != http.StatusOK || out["sealed"] != true {
		t.Fatalf("reloaded: code=%d out=%v", code, out)
	}
	if obs := out["observations"].([]interface{}); len(obs) != 1 {
		t.Fatalf("reloaded observations = %v", obs)
	}
	// Still sealed after restart.
	code, _ = doReq(t, h2, http.MethodPost, "/executions/"+id+"/observations", `{"observations": [
		{"vehicle_id": "a", "time": 1, "cell": {"x": 1, "y": 0}}]}`)
	if code != http.StatusConflict {
		t.Fatalf("post-restart report: code=%d", code)
	}
}

func TestExecutionLimit(t *testing.T) {
	store := newExecStore("", 1, 0)
	h := newRouterWithStore(store)
	createExec(t, h)
	code, out := doReq(t, h, http.MethodPost, "/executions", execCreateBody)
	if code != http.StatusTooManyRequests || out["error"] == nil {
		t.Fatalf("limit: code=%d out=%v", code, out)
	}
}

func TestBatchLimit(t *testing.T) {
	store := newExecStore("", 0, 2)
	h := newRouterWithStore(store)
	id := createExec(t, h)
	body := `{"observations": [`
	for i := 0; i < 3; i++ {
		if i > 0 {
			body += ","
		}
		body += fmt.Sprintf(`{"vehicle_id": "a", "time": %d, "cell": {"x": 0, "y": 0}}`, i)
	}
	body += "]}"
	code, _ := doReq(t, h, http.MethodPost, "/executions/"+id+"/observations", body)
	if code != http.StatusBadRequest {
		t.Fatalf("batch limit: code=%d", code)
	}
}
