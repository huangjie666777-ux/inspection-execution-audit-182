package main

import (
	"context"
	"testing"
)

// newTestExecution builds a solved execution for the shared two-task
// request so verification tests can feed observations into it.
func newTestExecution(t *testing.T) *Execution {
	t.Helper()
	req := validInspectReq()
	grid, err := validateInspectRequest(req)
	if err != nil {
		t.Fatalf("validate: %v", err)
	}
	assign, paths, arrivals, dwellStarts, cost, status := solveInspect(
		context.Background(), grid, req.Vehicles, req.Tasks, req.MaxSteps, req.NodeBudget)
	if status != StatusSuccess {
		t.Fatalf("solve: %s", status)
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
	return &Execution{
		ID:      "exec-test",
		Request: *req,
		Plan:    plan,
		obs:     map[string]map[int]Cell{},
	}
}

// observePlanned records the planned position of every vehicle for
// every time in 0..max_steps (the route extended by the goal after
// parking).
func observePlanned(e *Execution) {
	for _, v := range e.Request.Vehicles {
		for ts := 0; ts <= e.Request.MaxSteps; ts++ {
			if e.obs[v.ID] == nil {
				e.obs[v.ID] = map[int]Cell{}
			}
			e.obs[v.ID][ts] = e.plannedPos(v.ID, ts)
		}
	}
}

func TestVerifyPassedOnPlan(t *testing.T) {
	e := newTestExecution(t)
	observePlanned(e)
	rep := e.verify()
	if rep.Status != VerifyPassed {
		t.Fatalf("status = %s, violations = %v, tasks = %v", rep.Status, rep.Violations, rep.Tasks)
	}
	if len(rep.Deviations) != 0 || len(rep.Missing) != 0 {
		t.Fatalf("unexpected deviations/missing: %v %v", rep.Deviations, rep.Missing)
	}
	for _, tc := range rep.Tasks {
		if tc.Status != TaskVerified || tc.DwellStart == nil {
			t.Fatalf("task not verified: %+v", tc)
		}
		if *tc.DwellFinish-*tc.DwellStart != taskDwell(t, e, tc.TaskID) {
			t.Fatalf("bad dwell span: %+v", tc)
		}
	}
}

func taskDwell(t *testing.T, e *Execution, id string) int {
	t.Helper()
	for _, task := range e.Request.Tasks {
		if task.ID == id {
			return task.Dwell
		}
	}
	t.Fatalf("unknown task %s", id)
	return 0
}

func TestVerifyMissingIntervals(t *testing.T) {
	e := newTestExecution(t)
	observePlanned(e)
	// Drop a gap and the tail for vehicle a.
	delete(e.obs["a"], 3)
	delete(e.obs["a"], 4)
	for ts := 7; ts <= e.Request.MaxSteps; ts++ {
		delete(e.obs["a"], ts)
	}
	rep := e.verify()
	if rep.Status != VerifyUnknown {
		t.Fatalf("status = %s", rep.Status)
	}
	var gaps []MissingInterval
	for _, m := range rep.Missing {
		if m.VehicleID == "a" {
			gaps = append(gaps, m)
		}
	}
	if len(gaps) != 2 || gaps[0].From != 3 || gaps[0].To != 4 ||
		gaps[1].From != 7 || gaps[1].To != e.Request.MaxSteps {
		t.Fatalf("gaps = %v", gaps)
	}
	// The task of vehicle a cannot be proven nor disproven.
	for _, tc := range rep.Tasks {
		if tc.VehicleID == "a" && tc.Status != TaskUnknown {
			t.Fatalf("task = %+v", tc)
		}
	}
}

func TestVerifyIllegalMove(t *testing.T) {
	e := newTestExecution(t)
	observePlanned(e)
	e.obs["a"][5] = Cell{X: 0, Y: 1} // teleport away from the plan
	rep := e.verify()
	if rep.Status != VerifyFailed {
		t.Fatalf("status = %s", rep.Status)
	}
	found := false
	for _, v := range rep.Violations {
		if v.Type == ViolIllegalMove && v.Vehicles[0] == "a" {
			found = true
		}
	}
	if !found {
		t.Fatalf("no illegal_move violation: %v", rep.Violations)
	}
	if len(rep.Deviations) == 0 {
		t.Fatal("expected a deviation")
	}
}

func TestVerifyObstacleEvidence(t *testing.T) {
	e := newTestExecution(t)
	e.Request.Obstacles = []Cell{{X: 2, Y: 1}}
	observePlanned(e)
	e.obs["b"][6] = Cell{X: 2, Y: 1} // on the obstacle
	rep := e.verify()
	if rep.Status != VerifyFailed {
		t.Fatalf("status = %s", rep.Status)
	}
	found := false
	for _, v := range rep.Violations {
		if v.Type == ViolObstacle && v.Cell == (Cell{X: 2, Y: 1}) {
			found = true
		}
	}
	if !found {
		t.Fatalf("no obstacle violation: %v", rep.Violations)
	}
	// The evidence observation must be retained.
	if e.obs["b"][6] != (Cell{X: 2, Y: 1}) {
		t.Fatal("evidence observation dropped")
	}
}

func TestVerifySameCellAndSwap(t *testing.T) {
	e := newTestExecution(t)
	observePlanned(e)
	// Force a head-on swap between t=2 and t=3.
	e.obs["a"][2] = Cell{X: 1, Y: 0}
	e.obs["b"][2] = Cell{X: 2, Y: 0}
	e.obs["a"][3] = Cell{X: 2, Y: 0}
	e.obs["b"][3] = Cell{X: 1, Y: 0}
	rep := e.verify()
	var swap, sameCell bool
	for _, v := range rep.Violations {
		switch v.Type {
		case ViolHeadOnSwap:
			swap = true
			if len(v.Vehicles) != 2 {
				t.Fatalf("swap vehicles = %v", v.Vehicles)
			}
		case ViolSameCell:
			sameCell = true
		}
	}
	if !swap {
		t.Fatalf("no head_on_swap: %v", rep.Violations)
	}
	if sameCell {
		t.Fatal("unexpected same_cell")
	}
	// Now force a same-cell collision.
	e2 := newTestExecution(t)
	observePlanned(e2)
	e2.obs["b"][4] = e2.obs["a"][4]
	rep2 := e2.verify()
	found := false
	for _, v := range rep2.Violations {
		if v.Type == ViolSameCell {
			found = true
		}
	}
	if !found {
		t.Fatalf("no same_cell: %v", rep2.Violations)
	}
}

func TestVerifyStartMismatch(t *testing.T) {
	e := newTestExecution(t)
	observePlanned(e)
	e.obs["a"][0] = Cell{X: 1, Y: 0}
	rep := e.verify()
	found := false
	for _, v := range rep.Violations {
		if v.Type == ViolStartMismatch && v.Time == 0 {
			found = true
		}
	}
	if !found {
		t.Fatalf("no start_mismatch: %v", rep.Violations)
	}
}

func TestVerifyTaskIncomplete(t *testing.T) {
	e := newTestExecution(t)
	observePlanned(e)
	// Vehicle a fully observed but never on its job cell: it waited at
	// its start forever.
	start := e.Request.Vehicles[0].Start
	for ts := 0; ts <= e.Request.MaxSteps; ts++ {
		e.obs["a"][ts] = start
	}
	rep := e.verify()
	if rep.Status != VerifyFailed {
		t.Fatalf("status = %s", rep.Status)
	}
	for _, tc := range rep.Tasks {
		if tc.VehicleID == "a" && tc.Status != TaskIncomplete {
			t.Fatalf("task = %+v", tc)
		}
	}
}

func TestVerifyDwellWindowEarliest(t *testing.T) {
	e := newTestExecution(t)
	observePlanned(e)
	// Shift vehicle a's dwell later inside the window; the report must
	// name the earliest actually proven dwell span.
	var task TaskSpec
	for _, a := range e.Plan.Assignments {
		if a.VehicleID == "a" {
			for _, ts := range e.Request.Tasks {
				if ts.ID == a.TaskID {
					task = ts
				}
			}
		}
	}
	s := task.EarliestStart + 1
	for ts := s; ts <= s+task.Dwell; ts++ {
		e.obs["a"][ts] = task.Cell
	}
	rep := e.verify()
	for _, tc := range rep.Tasks {
		if tc.VehicleID != "a" {
			continue
		}
		if tc.Status != TaskVerified {
			t.Fatalf("task = %+v", tc)
		}
		if *tc.DwellStart > s || *tc.DwellFinish != *tc.DwellStart+task.Dwell {
			t.Fatalf("dwell span = %d..%d, want start <= %d",
				*tc.DwellStart, *tc.DwellFinish, s)
		}
	}
}
