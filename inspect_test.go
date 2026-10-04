package main

import (
	"context"
	"testing"
)

func validInspectReq() *InspectRequest {
	return &InspectRequest{
		Width: 6, Height: 2, MaxSteps: 16, NodeBudget: 20000,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{5, 0}},
			{ID: "b", Start: Cell{1, 1}, Goal: Cell{0, 1}},
		},
		Tasks: []TaskSpec{
			{ID: "t1", Cell: Cell{1, 0}, Vehicles: []string{"a", "b"}, EarliestStart: 0, LatestFinish: 12, Dwell: 1},
			{ID: "t2", Cell: Cell{4, 0}, Vehicles: []string{"a", "b"}, EarliestStart: 0, LatestFinish: 12, Dwell: 1},
		},
	}
}

func solveInspectReq(t *testing.T, req *InspectRequest) ([]int, [][]Cell, []int, []int, int, string) {
	t.Helper()
	grid, err := validateInspectRequest(req)
	if err != nil {
		t.Fatalf("validate: %v", err)
	}
	return solveInspect(context.Background(), grid, req.Vehicles, req.Tasks, req.MaxSteps, req.NodeBudget)
}

// checkInspect simulates the joint plan and asserts all movement,
// dwell, window and parking rules.
func checkInspect(t *testing.T, req *InspectRequest, assign []int, paths [][]Cell, arrivals, dwellStarts []int, cost int) {
	t.Helper()
	grid, _ := validateInspectRequest(req)
	seenTasks := map[int]bool{}
	sum := 0
	for i, p := range paths {
		v := req.Vehicles[i]
		task := req.Tasks[assign[i]]
		if seenTasks[assign[i]] {
			t.Fatalf("task %d assigned twice", assign[i])
		}
		seenTasks[assign[i]] = true
		exec := false
		for _, vid := range task.Vehicles {
			if vid == v.ID {
				exec = true
			}
		}
		if !exec {
			t.Errorf("vehicle %s cannot execute task %s", v.ID, task.ID)
		}
		if p[0] != v.Start || p[len(p)-1] != v.Goal {
			t.Errorf("vehicle %d endpoints mismatch", i)
		}
		if arrivals[i] != len(p)-1 || arrivals[i] > req.MaxSteps {
			t.Errorf("vehicle %d parking time mismatch", i)
		}
		sum += arrivals[i]
		ds, df := dwellStarts[i], dwellStarts[i]+task.Dwell
		if ds < task.EarliestStart || df > task.LatestFinish {
			t.Errorf("vehicle %d dwell [%d,%d] outside window [%d,%d]",
				i, ds, df, task.EarliestStart, task.LatestFinish)
		}
		// Dwell: at the job cell, uninterruptible, for [ds, df].
		for ts := ds; ts <= df; ts++ {
			if posAt(p, ts) != task.Cell {
				t.Errorf("vehicle %d not on job cell at t=%d during dwell", i, ts)
			}
		}
		// Before the dwell finishes the vehicle must not park at goal
		// unless goal == job cell; after df it heads to goal and stays.
		for k := 1; k < len(p); k++ {
			d := abs(p[k].X-p[k-1].X) + abs(p[k].Y-p[k-1].Y)
			if d > 1 {
				t.Errorf("vehicle %d teleports at step %d", i, k)
			}
			if !grid.free(p[k]) {
				t.Errorf("vehicle %d enters obstacle/out of bounds at step %d", i, k)
			}
		}
	}
	if sum != cost {
		t.Errorf("cost = %d, want sum of parking times %d", cost, sum)
	}
	for ts := 0; ts <= req.MaxSteps; ts++ {
		for i := 0; i < len(paths); i++ {
			for j := i + 1; j < len(paths); j++ {
				if posAt(paths[i], ts) == posAt(paths[j], ts) {
					t.Fatalf("vertex conflict between %d and %d at t=%d", i, j, ts)
				}
				if ts > 0 {
					a0, a1 := posAt(paths[i], ts-1), posAt(paths[i], ts)
					b0, b1 := posAt(paths[j], ts-1), posAt(paths[j], ts)
					if a0 == b1 && b0 == a1 && a0 != a1 {
						t.Fatalf("edge swap between %d and %d at t=%d", i, j, ts)
					}
				}
			}
		}
	}
}

// The distance-greedy assignment (a->t1, b->t2) costs 16; the joint
// optimum swaps the tasks for a total cost of 11.
func TestInspectBeatsDistanceGreedy(t *testing.T) {
	req := validInspectReq()
	assign, paths, arrivals, dwellStarts, cost, status := solveInspectReq(t, req)
	if status != StatusSuccess {
		t.Fatalf("status = %s", status)
	}
	if cost != 11 {
		t.Fatalf("cost = %d, want 11 (arrivals %v)", cost, arrivals)
	}
	if req.Tasks[assign[0]].ID != "t2" || req.Tasks[assign[1]].ID != "t1" {
		t.Fatalf("assignment = %v, want a->t2, b->t1", assign)
	}
	checkInspect(t, req, assign, paths, arrivals, dwellStarts, cost)
}

func TestInspectSingleVehicle(t *testing.T) {
	req := &InspectRequest{
		Width: 4, Height: 1, MaxSteps: 16, NodeBudget: 100,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{3, 0}},
		},
		Tasks: []TaskSpec{
			{ID: "t1", Cell: Cell{1, 0}, Vehicles: []string{"a"}, EarliestStart: 2, LatestFinish: 8, Dwell: 2},
		},
	}
	assign, paths, arrivals, dwellStarts, cost, status := solveInspectReq(t, req)
	if status != StatusSuccess {
		t.Fatalf("status = %s", status)
	}
	// Arrive at job cell t=1, wait, dwell [2,4], then 2 steps to goal.
	if dwellStarts[0] != 2 || arrivals[0] != 6 || cost != 6 {
		t.Fatalf("dwellStart=%v arrivals=%v cost=%d", dwellStarts, arrivals, cost)
	}
	checkInspect(t, req, assign, paths, arrivals, dwellStarts, cost)
}

// Window shorter than the dwell is infeasible, hence no_solution.
func TestInspectWindowTooShort(t *testing.T) {
	req := validInspectReq()
	req.Tasks[0].EarliestStart = 2
	req.Tasks[0].LatestFinish = 3
	req.Tasks[0].Dwell = 2
	_, _, _, _, _, status := solveInspectReq(t, req)
	if status != StatusNoSolution {
		t.Fatalf("status = %s, want no_solution", status)
	}
}

// Executable sets force the assignment even against distance.
func TestInspectForcedAssignment(t *testing.T) {
	req := validInspectReq()
	req.Tasks[0].Vehicles = []string{"b"}
	req.Tasks[1].Vehicles = []string{"a"}
	assign, paths, arrivals, dwellStarts, cost, status := solveInspectReq(t, req)
	if status != StatusSuccess {
		t.Fatalf("status = %s", status)
	}
	if req.Tasks[assign[0]].ID != "t2" || req.Tasks[assign[1]].ID != "t1" {
		t.Fatalf("assignment = %v, want a->t2, b->t1", assign)
	}
	checkInspect(t, req, assign, paths, arrivals, dwellStarts, cost)
}

// No legal assignment exists when a task's executable set cannot be
// matched one-to-one.
func TestInspectNoLegalAssignment(t *testing.T) {
	req := validInspectReq()
	req.Tasks[0].Vehicles = []string{"a"}
	req.Tasks[1].Vehicles = []string{"a"}
	_, _, _, _, _, status := solveInspectReq(t, req)
	if status != StatusNoSolution {
		t.Fatalf("status = %s, want no_solution", status)
	}
}

// Budget 1 cannot resolve the head-on coordination: undecided.
func TestInspectBudgetUndecided(t *testing.T) {
	req := validInspectReq()
	req.NodeBudget = 1
	_, _, _, _, _, status := solveInspectReq(t, req)
	if status != StatusUndecided {
		t.Fatalf("status = %s, want undecided", status)
	}
}

func TestInspectCancelled(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	req := validInspectReq()
	grid, _ := validateInspectRequest(req)
	_, _, _, _, _, status := solveInspect(ctx, grid, req.Vehicles, req.Tasks, req.MaxSteps, req.NodeBudget)
	if status != StatusUndecided {
		t.Fatalf("status = %s, want undecided on cancel", status)
	}
}

// A vehicle parked from time 0 (start == goal) must not drive away;
// the other vehicle detours around it.
func TestStartEqualsGoalStaysParked(t *testing.T) {
	req := &PlanRequest{
		Width: 3, Height: 2, MaxSteps: 16, NodeBudget: 20000,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{1, 0}, Goal: Cell{1, 0}},
			{ID: "b", Start: Cell{0, 0}, Goal: Cell{2, 0}},
		},
	}
	paths, arrivals, cost, status := solve(t, req)
	if status != StatusSuccess {
		t.Fatalf("status = %s", status)
	}
	if arrivals[0] != 0 || len(paths[0]) != 1 {
		t.Fatalf("parked vehicle moved: arrivals=%v path=%v", arrivals, paths[0])
	}
	if arrivals[1] != 4 || cost != 4 {
		t.Fatalf("arrivals=%v cost=%d, want detour cost 4", arrivals, cost)
	}
	checkPlan(t, req, paths, arrivals, req.MaxSteps)
}

func TestInspectValidateRejects(t *testing.T) {
	cases := map[string]func(*InspectRequest){
		"four vehicles": func(r *InspectRequest) {
			r.Vehicles = append(r.Vehicles, VehicleSpec{ID: "c", Start: Cell{5, 1}, Goal: Cell{3, 1}})
		},
		"task count mismatch": func(r *InspectRequest) { r.Tasks = r.Tasks[:1] },
		"empty task id":       func(r *InspectRequest) { r.Tasks[0].ID = "" },
		"dup task id":         func(r *InspectRequest) { r.Tasks[1].ID = "t1" },
		"task on obstacle": func(r *InspectRequest) {
			r.Obstacles = []Cell{{2, 0}}
			r.Tasks[0].Cell = Cell{2, 0}
		},
		"task cell oob":      func(r *InspectRequest) { r.Tasks[0].Cell = Cell{9, 0} },
		"empty executables":  func(r *InspectRequest) { r.Tasks[0].Vehicles = nil },
		"unknown vehicle":    func(r *InspectRequest) { r.Tasks[0].Vehicles = []string{"zz"} },
		"dup executable":     func(r *InspectRequest) { r.Tasks[0].Vehicles = []string{"a", "a"} },
		"negative start":     func(r *InspectRequest) { r.Tasks[0].EarliestStart = -1 },
		"finish too big":     func(r *InspectRequest) { r.Tasks[0].LatestFinish = 65 },
		"inverted window":    func(r *InspectRequest) { r.Tasks[0].EarliestStart = 5; r.Tasks[0].LatestFinish = 4 },
		"dwell zero":         func(r *InspectRequest) { r.Tasks[0].Dwell = 0 },
		"dwell too big":      func(r *InspectRequest) { r.Tasks[0].Dwell = 9 },
		"start on obstacle":  func(r *InspectRequest) { r.Obstacles = []Cell{{0, 0}} },
		"duplicate start":    func(r *InspectRequest) { r.Vehicles[1].Start = r.Vehicles[0].Start },
		"steps out of range": func(r *InspectRequest) { r.MaxSteps = 65 },
	}
	for name, mutate := range cases {
		req := validInspectReq()
		mutate(req)
		if _, err := validateInspectRequest(req); err == nil {
			t.Errorf("%s: expected error, got nil", name)
		}
	}
}
