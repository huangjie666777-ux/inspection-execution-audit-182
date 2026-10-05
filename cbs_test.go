package main

import (
	"context"
	"testing"
)

func solve(t *testing.T, req *PlanRequest) ([][]Cell, []int, int, string) {
	t.Helper()
	grid, err := validateRequest(req)
	if err != nil {
		t.Fatalf("validate: %v", err)
	}
	return solveCBS(context.Background(), grid, req.Vehicles, req.MaxSteps, req.NodeBudget)
}

// checkPlan simulates the joint plan and asserts all movement rules.
func checkPlan(t *testing.T, req *PlanRequest, paths [][]Cell, arrivals []int, maxSteps int) {
	t.Helper()
	grid, _ := validateRequest(req)
	for i, p := range paths {
		if p[0] != req.Vehicles[i].Start {
			t.Errorf("vehicle %d does not start at its start cell", i)
		}
		if p[len(p)-1] != req.Vehicles[i].Goal {
			t.Errorf("vehicle %d does not end at its goal", i)
		}
		if arrivals[i] != len(p)-1 {
			t.Errorf("vehicle %d arrival mismatch", i)
		}
		if arrivals[i] > maxSteps {
			t.Errorf("vehicle %d exceeds max steps", i)
		}
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
	for tstep := 0; tstep <= maxSteps; tstep++ {
		for i := 0; i < len(paths); i++ {
			for j := i + 1; j < len(paths); j++ {
				if posAt(paths[i], tstep) == posAt(paths[j], tstep) {
					t.Fatalf("vertex conflict between %d and %d at t=%d", i, j, tstep)
				}
				if tstep > 0 {
					a0, a1 := posAt(paths[i], tstep-1), posAt(paths[i], tstep)
					b0, b1 := posAt(paths[j], tstep-1), posAt(paths[j], tstep)
					if a0 == b1 && b0 == a1 && a0 != a1 {
						t.Fatalf("edge swap between %d and %d at t=%d", i, j, tstep)
					}
				}
			}
		}
	}
}

func TestSingleVehicleStraightLine(t *testing.T) {
	req := &PlanRequest{
		Width: 5, Height: 1, MaxSteps: 10, NodeBudget: 100,
		Vehicles: []VehicleSpec{{ID: "a", Start: Cell{0, 0}, Goal: Cell{4, 0}}},
	}
	paths, arrivals, cost, status := solve(t, req)
	if status != StatusSuccess {
		t.Fatalf("status = %s", status)
	}
	if arrivals[0] != 4 || cost != 4 || len(paths[0]) != 5 {
		t.Fatalf("got arrivals=%v cost=%d path=%v", arrivals, cost, paths[0])
	}
	checkPlan(t, req, paths, arrivals, req.MaxSteps)
}

func TestStartEqualsGoal(t *testing.T) {
	req := &PlanRequest{
		Width: 3, Height: 3, MaxSteps: 10, NodeBudget: 100,
		Vehicles: []VehicleSpec{{ID: "a", Start: Cell{1, 1}, Goal: Cell{1, 1}}},
	}
	_, arrivals, cost, status := solve(t, req)
	if status != StatusSuccess || arrivals[0] != 0 || cost != 0 {
		t.Fatalf("status=%s arrivals=%v cost=%d", status, arrivals, cost)
	}
}

func TestDetourAroundObstacle(t *testing.T) {
	req := &PlanRequest{
		Width: 3, Height: 2, MaxSteps: 10, NodeBudget: 100,
		Obstacles: []Cell{{1, 0}},
		Vehicles:  []VehicleSpec{{ID: "a", Start: Cell{0, 0}, Goal: Cell{2, 0}}},
	}
	paths, arrivals, _, status := solve(t, req)
	if status != StatusSuccess || arrivals[0] != 4 {
		t.Fatalf("status=%s arrivals=%v", status, arrivals)
	}
	checkPlan(t, req, paths, arrivals, req.MaxSteps)
}

// Two vehicles swap ends of a 2x2 block: one must detour, total cost 4.
func TestCoordinatedSwap(t *testing.T) {
	req := &PlanRequest{
		Width: 2, Height: 2, MaxSteps: 16, NodeBudget: 1000,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{1, 0}},
			{ID: "b", Start: Cell{1, 0}, Goal: Cell{0, 0}},
		},
	}
	paths, arrivals, cost, status := solve(t, req)
	if status != StatusSuccess {
		t.Fatalf("status = %s", status)
	}
	if cost != 4 {
		t.Fatalf("cost = %d, want 4 (arrivals %v)", cost, arrivals)
	}
	checkPlan(t, req, paths, arrivals, req.MaxSteps)
}

// A vehicle already parked on its goal must still be avoided by others.
func TestPostArrivalOccupancy(t *testing.T) {
	req := &PlanRequest{
		Width: 3, Height: 1, MaxSteps: 8, NodeBudget: 20000,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{2, 0}},
			{ID: "b", Start: Cell{1, 0}, Goal: Cell{1, 0}},
		},
	}
	_, _, _, status := solve(t, req)
	if status != StatusNoSolution {
		t.Fatalf("status = %s, want no_solution (b blocks the only corridor)", status)
	}
}

// Swapping in a 1-wide corridor is impossible: search must exhaust.
func TestCorridorSwapNoSolution(t *testing.T) {
	req := &PlanRequest{
		Width: 2, Height: 1, MaxSteps: 12, NodeBudget: 20000,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{1, 0}},
			{ID: "b", Start: Cell{1, 0}, Goal: Cell{0, 0}},
		},
	}
	_, _, _, status := solve(t, req)
	if status != StatusNoSolution {
		t.Fatalf("status = %s, want no_solution", status)
	}
}

// A 3x2 head-on swap needs several expansions; budget 1 must be undecided.
func TestBudgetExhaustedUndecided(t *testing.T) {
	req := &PlanRequest{
		Width: 3, Height: 2, MaxSteps: 16, NodeBudget: 1,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{2, 0}},
			{ID: "b", Start: Cell{2, 0}, Goal: Cell{0, 0}},
		},
	}
	_, _, _, status := solve(t, req)
	if status != StatusUndecided {
		t.Fatalf("status = %s, want undecided", status)
	}
}

// Same scenario with ample budget succeeds and is conflict-free.
func TestHeadOnSwapSolved(t *testing.T) {
	req := &PlanRequest{
		Width: 3, Height: 2, MaxSteps: 16, NodeBudget: 20000,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{2, 0}},
			{ID: "b", Start: Cell{2, 0}, Goal: Cell{0, 0}},
		},
	}
	paths, arrivals, cost, status := solve(t, req)
	if status != StatusSuccess {
		t.Fatalf("status = %s", status)
	}
	if cost != 6 {
		t.Fatalf("cost = %d, want 6 (arrivals %v)", cost, arrivals)
	}
	checkPlan(t, req, paths, arrivals, req.MaxSteps)
}

func TestUnreachableGoal(t *testing.T) {
	req := &PlanRequest{
		Width: 2, Height: 2, MaxSteps: 16, NodeBudget: 100,
		Obstacles: []Cell{{1, 0}, {0, 1}},
		Vehicles:  []VehicleSpec{{ID: "a", Start: Cell{0, 0}, Goal: Cell{1, 1}}},
	}
	_, _, _, status := solve(t, req)
	if status != StatusNoSolution {
		t.Fatalf("status = %s, want no_solution", status)
	}
}

func TestCancelledContext(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	req := &PlanRequest{
		Width: 3, Height: 2, MaxSteps: 16, NodeBudget: 20000,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{2, 0}},
			{ID: "b", Start: Cell{2, 0}, Goal: Cell{0, 0}},
		},
	}
	grid, _ := validateRequest(req)
	_, _, _, status := solveCBS(ctx, grid, req.Vehicles, req.MaxSteps, req.NodeBudget)
	if status != StatusUndecided {
		t.Fatalf("status = %s, want undecided on cancel", status)
	}
}
