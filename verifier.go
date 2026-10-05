package main

import "sort"

func cellPtr(c Cell) *Cell { return &c }

func plannedCell(route []Cell, parkingTime, t int) Cell {
	if t <= parkingTime {
		return route[t]
	}
	return route[parkingTime]
}

func inBounds(plan ExecutionPlan, c Cell) bool {
	return c.X >= 0 && c.X < plan.Width && c.Y >= 0 && c.Y < plan.Height
}

func missingIntervals(vehicleID string, points []ObservedPoint, horizon int) []MissingInterval {
	present := map[int]bool{}
	for _, p := range points {
		present[p.Time] = true
	}
	var out []MissingInterval
	for t := 0; t <= horizon; {
		if present[t] {
			t++
			continue
		}
		start := t
		for t <= horizon && !present[t] {
			t++
		}
		out = append(out, MissingInterval{VehicleID: vehicleID, Start: start, End: t - 1})
	}
	return out
}

func evaluateTasks(plan ExecutionPlan, byVehicle map[string]map[int]Cell, complete map[string]bool) ([]TaskResult, bool) {
	results := make([]TaskResult, 0, len(plan.Vehicles))
	anyIncomplete := false
	taskByID := map[string]TaskSpec{}
	for _, task := range plan.Tasks {
		taskByID[task.ID] = task
	}
	for _, pv := range plan.Vehicles {
		task := taskByID[pv.TaskID]
		seen := byVehicle[pv.VehicleID]
		status := TaskUnknown
		var bestStart, bestFinish int
		found := false
		last := task.LatestFinish - task.Dwell
		if last > plan.MaxSteps {
			last = plan.MaxSteps
		}
		for start := task.EarliestStart; start <= last; start++ {
			finish := start + task.Dwell
			ok := true
			for t := start; t <= finish; t++ {
				c, exists := seen[t]
				if !exists || c != task.Cell {
					ok = false
					break
				}
			}
			if ok {
				found = true
				bestStart, bestFinish = start, finish
				break
			}
		}
		if found {
			status = TaskCompleted
		} else if complete[pv.VehicleID] {
			status = TaskIncomplete
			anyIncomplete = true
		}
		result := TaskResult{TaskID: task.ID, VehicleID: pv.VehicleID, Status: status}
		if found {
			result.EarliestStart = bestStart
			result.EarliestFinish = bestFinish
		}
		results = append(results, result)
	}
	return results, anyIncomplete
}

func buildReport(rec *ExecutionRecord) Report {
	plan := rec.Plan
	horizon := plan.MaxSteps
	blocked := map[Cell]bool{}
	for _, o := range plan.Obstacles {
		blocked[o] = true
	}
	byVehicle := map[string]map[int]Cell{}
	pointsByVehicle := map[string][]ObservedPoint{}
	complete := map[string]bool{}
	missing := []MissingInterval{}
	evidence := []Evidence{}
	deviations := []Evidence{}
	total := 0

	for _, pv := range plan.Vehicles {
		points := append([]ObservedPoint(nil), rec.Observations[pv.VehicleID]...)
		sort.Slice(points, func(i, j int) bool { return points[i].Time < points[j].Time })
		pointsByVehicle[pv.VehicleID] = points
		seen := map[int]Cell{}
		for _, p := range points {
			seen[p.Time] = p.Cell
		}
		byVehicle[pv.VehicleID] = seen
		total += len(points)
		complete[pv.VehicleID] = len(points) == horizon+1
		vehicleMissing := missingIntervals(pv.VehicleID, points, horizon)
		missing = append(missing, vehicleMissing...)

		for _, p := range points {
			expected := plannedCell(pv.Path, pv.ParkingTime, p.Time)
			if p.Cell != expected {
				kind := "plan_deviation"
				if p.Time == 0 {
					kind = "start_mismatch"
				}
				ev := Evidence{Type: kind, VehicleID: pv.VehicleID, Time: p.Time,
					Expected: cellPtr(expected), Actual: cellPtr(p.Cell)}
				evidence = append(evidence, ev)
				deviations = append(deviations, ev)
			}
			if inBounds(plan, p.Cell) && blocked[p.Cell] {
				evidence = append(evidence, Evidence{Type: "obstacle", VehicleID: pv.VehicleID,
					Time: p.Time, Cell: cellPtr(p.Cell)})
			}
		}
		for i := 1; i < len(points); i++ {
			a, b := points[i-1], points[i]
			if b.Time != a.Time+1 {
				continue
			}
			distance := abs(b.Cell.X-a.Cell.X) + abs(b.Cell.Y-a.Cell.Y)
			if distance > 1 {
				evidence = append(evidence, Evidence{Type: "movement_violation", VehicleID: pv.VehicleID,
					FromTime: a.Time, ToTime: b.Time, From: cellPtr(a.Cell), To: cellPtr(b.Cell)})
			}
		}
	}

	for i := 0; i < len(plan.Vehicles); i++ {
		for j := i + 1; j < len(plan.Vehicles); j++ {
			a, b := plan.Vehicles[i].VehicleID, plan.Vehicles[j].VehicleID
			for t := 0; t <= horizon; t++ {
				ca, oa := byVehicle[a][t]
				cb, ob := byVehicle[b][t]
				if oa && ob && ca == cb {
					evidence = append(evidence, Evidence{Type: "vertex_conflict",
						VehicleIDs: []string{a, b}, Time: t, Cell: cellPtr(ca)})
				}
				if t > 0 {
					a0, a1 := byVehicle[a][t-1], byVehicle[a][t]
					b0, b1 := byVehicle[b][t-1], byVehicle[b][t]
					_, p00 := byVehicle[a][t-1]
					_, p11 := byVehicle[a][t]
					_, p01 := byVehicle[b][t-1]
					_, p10 := byVehicle[b][t]
					if p00 && p11 && p01 && p10 && a0 == b1 && b0 == a1 && a0 != a1 {
						evidence = append(evidence, Evidence{Type: "edge_swap",
							VehicleIDs: []string{a, b}, FromTime: t - 1, ToTime: t,
							From: cellPtr(a0), To: cellPtr(a1)})
					}
				}
			}
		}
	}

	taskResults, incomplete := evaluateTasks(plan, byVehicle, complete)
	hasMissing := len(missing) > 0
	status := ReportPassed
	if len(evidence) > 0 || incomplete {
		status = ReportFailed
	} else if hasMissing {
		status = ReportUnknown
	}
	if evidence == nil {
		evidence = []Evidence{}
	}
	if deviations == nil {
		deviations = []Evidence{}
	}
	if missing == nil {
		missing = []MissingInterval{}
	}
	sortEvidence(evidence)
	sortEvidence(deviations)
	return Report{
		Status: status, ObservationCount: total, DeviationCount: len(deviations),
		Deviations: deviations, MissingIntervals: missing,
		TaskResults: taskResults, Evidence: evidence,
	}
}

func sortEvidence(items []Evidence) {
	sort.Slice(items, func(i, j int) bool {
		a, b := items[i], items[j]
		if a.Time != b.Time {
			return a.Time < b.Time
		}
		if a.VehicleID != b.VehicleID {
			return a.VehicleID < b.VehicleID
		}
		return a.Type < b.Type
	})
}
