package main

import (
	"errors"
	"fmt"
)

// checkMapLimits validates the shared scalar limits.
func checkMapLimits(width, height, maxSteps, nodeBudget int) error {
	if width < 1 || width > maxGridDim || height < 1 || height > maxGridDim {
		return fmt.Errorf("width and height must be within 1..%d", maxGridDim)
	}
	if maxSteps < 1 || maxSteps > maxStepsLimit {
		return fmt.Errorf("max_steps must be within 1..%d", maxStepsLimit)
	}
	if nodeBudget < 1 || nodeBudget > maxBudget {
		return fmt.Errorf("node_budget must be within 1..%d", maxBudget)
	}
	return nil
}

// buildGrid validates obstacles and returns the shared map.
func buildGrid(width, height int, obstacles []Cell) (*Grid, error) {
	grid := &Grid{W: width, H: height, Blocked: map[Cell]bool{}}
	for _, o := range obstacles {
		if o.X < 0 || o.X >= width || o.Y < 0 || o.Y >= height {
			return nil, fmt.Errorf("obstacle (%d,%d) out of bounds", o.X, o.Y)
		}
		grid.Blocked[o] = true
	}
	return grid, nil
}

// checkVehicleList validates vehicle specs against the grid.
func checkVehicleList(grid *Grid, vehicles []VehicleSpec, maxN int) error {
	if len(vehicles) < 1 || len(vehicles) > maxN {
		return fmt.Errorf("vehicles must contain 1..%d entries", maxN)
	}

	ids := map[string]bool{}
	starts := map[Cell]bool{}
	goals := map[Cell]bool{}
	for _, v := range vehicles {
		if v.ID == "" {
			return errors.New("vehicle id must not be empty")
		}
		if ids[v.ID] {
			return fmt.Errorf("duplicate vehicle id %q", v.ID)
		}
		ids[v.ID] = true
		if !grid.free(v.Start) {
			return fmt.Errorf("vehicle %q start (%d,%d) out of bounds or on obstacle", v.ID, v.Start.X, v.Start.Y)
		}
		if !grid.free(v.Goal) {
			return fmt.Errorf("vehicle %q goal (%d,%d) out of bounds or on obstacle", v.ID, v.Goal.X, v.Goal.Y)
		}
		if starts[v.Start] {
			return fmt.Errorf("duplicate start cell (%d,%d)", v.Start.X, v.Start.Y)
		}
		starts[v.Start] = true
		if goals[v.Goal] {
			return fmt.Errorf("duplicate goal cell (%d,%d)", v.Goal.X, v.Goal.Y)
		}
		goals[v.Goal] = true
	}
	return nil
}

// validateRequest checks all request-level invariants and returns a Grid.
func validateRequest(req *PlanRequest) (*Grid, error) {
	if err := checkMapLimits(req.Width, req.Height, req.MaxSteps, req.NodeBudget); err != nil {
		return nil, err
	}
	grid, err := buildGrid(req.Width, req.Height, req.Obstacles)
	if err != nil {
		return nil, err
	}
	if err := checkVehicleList(grid, req.Vehicles, maxVehicles); err != nil {
		return nil, err
	}
	return grid, nil
}

// validateInspectRequest checks map, vehicles and tasks for POST /inspect.
func validateInspectRequest(req *InspectRequest) (*Grid, error) {
	if err := checkMapLimits(req.Width, req.Height, req.MaxSteps, req.NodeBudget); err != nil {
		return nil, err
	}
	grid, err := buildGrid(req.Width, req.Height, req.Obstacles)
	if err != nil {
		return nil, err
	}
	if err := checkVehicleList(grid, req.Vehicles, maxInspectVehicles); err != nil {
		return nil, err
	}
	if len(req.Tasks) != len(req.Vehicles) {
		return nil, fmt.Errorf("tasks must contain exactly %d entries (one per vehicle)", len(req.Vehicles))
	}

	known := map[string]bool{}
	for _, v := range req.Vehicles {
		known[v.ID] = true
	}
	ids := map[string]bool{}
	for _, task := range req.Tasks {
		if task.ID == "" {
			return nil, errors.New("task id must not be empty")
		}
		if ids[task.ID] {
			return nil, fmt.Errorf("duplicate task id %q", task.ID)
		}
		ids[task.ID] = true
		if !grid.free(task.Cell) {
			return nil, fmt.Errorf("task %q cell (%d,%d) out of bounds or on obstacle", task.ID, task.Cell.X, task.Cell.Y)
		}
		if len(task.Vehicles) == 0 {
			return nil, fmt.Errorf("task %q must list at least one executable vehicle", task.ID)
		}
		seen := map[string]bool{}
		for _, vid := range task.Vehicles {
			if !known[vid] {
				return nil, fmt.Errorf("task %q references unknown vehicle %q", task.ID, vid)
			}
			if seen[vid] {
				return nil, fmt.Errorf("task %q lists vehicle %q twice", task.ID, vid)
			}
			seen[vid] = true
		}
		if task.EarliestStart < 0 || task.EarliestStart > maxWindowTime ||
			task.LatestFinish < 0 || task.LatestFinish > maxWindowTime {
			return nil, fmt.Errorf("task %q window times must be within 0..%d", task.ID, maxWindowTime)
		}
		if task.EarliestStart > task.LatestFinish {
			return nil, fmt.Errorf("task %q window inverted: earliest_start %d > latest_finish %d",
				task.ID, task.EarliestStart, task.LatestFinish)
		}
		if task.Dwell < minDwell || task.Dwell > maxDwell {
			return nil, fmt.Errorf("task %q dwell must be within %d..%d", task.ID, minDwell, maxDwell)
		}
	}
	return grid, nil
}
