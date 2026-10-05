package main

import "fmt"

// Overall verification outcomes.
const (
	VerifyPassed  = "passed"
	VerifyFailed  = "failed"
	VerifyUnknown = "unknown"
)

// Violation types detected against the movement and collision rules.
const (
	ViolStartMismatch = "start_mismatch"
	ViolObstacle      = "obstacle"
	ViolIllegalMove   = "illegal_move"
	ViolSameCell      = "same_cell"
	ViolHeadOnSwap    = "head_on_swap"
)

// Task verification outcomes.
const (
	TaskVerified   = "verified"
	TaskIncomplete = "incomplete"
	TaskUnknown    = "unknown"
)

// Deviation reports one observation that differs from the frozen plan.
type Deviation struct {
	VehicleID string `json:"vehicle_id"`
	Time      int    `json:"time"`
	Planned   Cell   `json:"planned"`
	Observed  Cell   `json:"observed"`
}

// Violation reports one broken movement or collision rule, identifying
// the vehicles, the time and the position involved.
type Violation struct {
	Type     string   `json:"type"`
	Vehicles []string `json:"vehicles"`
	Time     int      `json:"time"`
	Cell     Cell     `json:"cell"`
	Detail   string   `json:"detail"`
}

// MissingInterval is a maximal run of consecutive times without any
// observation for one vehicle. Both bounds are inclusive.
type MissingInterval struct {
	VehicleID string `json:"vehicle_id"`
	From      int    `json:"from"`
	To        int    `json:"to"`
}

// TaskCheck reports the dwell verification of one assigned task.
// DwellStart/DwellFinish carry the earliest proven dwell window.
type TaskCheck struct {
	TaskID      string `json:"task_id"`
	VehicleID   string `json:"vehicle_id"`
	Status      string `json:"status"`
	DwellStart  *int   `json:"dwell_start,omitempty"`
	DwellFinish *int   `json:"dwell_finish,omitempty"`
}

// VerifyReport is the full comparison of the recorded observations
// against the frozen plan and the movement rules.
type VerifyReport struct {
	ExecutionID string            `json:"execution_id"`
	Sealed      bool              `json:"sealed"`
	Status      string            `json:"status"`
	Deviations  []Deviation       `json:"deviations"`
	Violations  []Violation       `json:"violations"`
	Missing     []MissingInterval `json:"missing"`
	Tasks       []TaskCheck       `json:"tasks"`
}

func manhattan(a, b Cell) int {
	dx := a.X - b.X
	if dx < 0 {
		dx = -dx
	}
	dy := a.Y - b.Y
	if dy < 0 {
		dy = -dy
	}
	return dx + dy
}

// verify compares all recorded observations against the frozen plan.
// Missing observations are reported as intervals and never
// interpolated: no planned or inferred positions are substituted.
func (e *Execution) verify() *VerifyReport {
	grid, _ := buildGrid(e.Request.Width, e.Request.Height, e.Request.Obstacles)
	maxSteps := e.Request.MaxSteps
	rep := &VerifyReport{
		ExecutionID: e.ID,
		Sealed:      e.Sealed,
		Deviations:  []Deviation{},
		Violations:  []Violation{},
		Missing:     []MissingInterval{},
		Tasks:       []TaskCheck{},
	}

	vehicleIDs := make([]string, len(e.Request.Vehicles))
	starts := map[string]Cell{}
	for i, v := range e.Request.Vehicles {
		vehicleIDs[i] = v.ID
		starts[v.ID] = v.Start
	}

	// Per-vehicle checks: start, obstacles, deviations, moves, gaps.
	for _, vid := range vehicleIDs {
		cells := e.obs[vid]
		if c, ok := cells[0]; ok && c != starts[vid] {
			rep.Violations = append(rep.Violations, Violation{
				Type:     ViolStartMismatch,
				Vehicles: []string{vid},
				Time:     0,
				Cell:     c,
				Detail: fmt.Sprintf("vehicle %q starts at (%d,%d), observed (%d,%d)",
					vid, starts[vid].X, starts[vid].Y, c.X, c.Y),
			})
		}
		for t := 0; t <= maxSteps; t++ {
			c, ok := cells[t]
			if !ok {
				continue
			}
			if grid.Blocked[c] {
				rep.Violations = append(rep.Violations, Violation{
					Type:     ViolObstacle,
					Vehicles: []string{vid},
					Time:     t,
					Cell:     c,
					Detail:   fmt.Sprintf("vehicle %q observed on obstacle (%d,%d)", vid, c.X, c.Y),
				})
			}
			if planned := e.plannedPos(vid, t); planned != c {
				rep.Deviations = append(rep.Deviations, Deviation{
					VehicleID: vid, Time: t, Planned: planned, Observed: c,
				})
			}
		}
		for t := 0; t < maxSteps; t++ {
			from, ok1 := cells[t]
			to, ok2 := cells[t+1]
			if !ok1 || !ok2 {
				continue
			}
			if manhattan(from, to) > 1 {
				rep.Violations = append(rep.Violations, Violation{
					Type:     ViolIllegalMove,
					Vehicles: []string{vid},
					Time:     t + 1,
					Cell:     to,
					Detail: fmt.Sprintf("vehicle %q moved from (%d,%d) at t=%d to (%d,%d) at t=%d",
						vid, from.X, from.Y, t, to.X, to.Y, t+1),
				})
			}
		}
		// Missing intervals over the full 0..max_steps horizon.
		gapStart := -1
		for t := 0; t <= maxSteps; t++ {
			if _, ok := cells[t]; ok {
				if gapStart >= 0 {
					rep.Missing = append(rep.Missing, MissingInterval{VehicleID: vid, From: gapStart, To: t - 1})
					gapStart = -1
				}
			} else if gapStart < 0 {
				gapStart = t
			}
		}
		if gapStart >= 0 {
			rep.Missing = append(rep.Missing, MissingInterval{VehicleID: vid, From: gapStart, To: maxSteps})
		}
	}

	// Cross-vehicle checks: same-cell occupancy and head-on swaps,
	// restricted to times where both vehicles were observed.
	for t := 0; t <= maxSteps; t++ {
		for i := 0; i < len(vehicleIDs); i++ {
			for j := i + 1; j < len(vehicleIDs); j++ {
				a, b := vehicleIDs[i], vehicleIDs[j]
				ca, oka := e.obs[a][t]
				cb, okb := e.obs[b][t]
				if oka && okb && ca == cb {
					rep.Violations = append(rep.Violations, Violation{
						Type:     ViolSameCell,
						Vehicles: []string{a, b},
						Time:     t,
						Cell:     ca,
						Detail:   fmt.Sprintf("vehicles %q and %q both occupy (%d,%d)", a, b, ca.X, ca.Y),
					})
				}
				if t < maxSteps {
					na, oka2 := e.obs[a][t+1]
					nb, okb2 := e.obs[b][t+1]
					if oka && okb && oka2 && okb2 && ca == nb && cb == na && ca != na {
						rep.Violations = append(rep.Violations, Violation{
							Type:     ViolHeadOnSwap,
							Vehicles: []string{a, b},
							Time:     t + 1,
							Cell:     na,
							Detail: fmt.Sprintf("vehicles %q and %q swapped (%d,%d)<->(%d,%d) between t=%d and t=%d",
								a, b, ca.X, ca.Y, cb.X, cb.Y, t, t+1),
						})
					}
				}
			}
		}
	}

	// Task dwell checks against the frozen assignment.
	taskByID := map[string]TaskSpec{}
	for _, task := range e.Request.Tasks {
		taskByID[task.ID] = task
	}
	for _, a := range e.Plan.Assignments {
		task := taskByID[a.TaskID]
		check := TaskCheck{TaskID: a.TaskID, VehicleID: a.VehicleID}
		cells := e.obs[a.VehicleID]
		// dwell+1 consecutive observed samples on the job cell, the
		// first at s and the last at s+dwell, prove dwell steps of
		// dwelling. The whole span must lie inside the window.
		for s := task.EarliestStart; s+task.Dwell <= task.LatestFinish && s+task.Dwell <= maxSteps; s++ {
			proven := true
			for k := 0; k <= task.Dwell; k++ {
				c, ok := cells[s+k]
				if !ok || c != task.Cell {
					proven = false
					break
				}
			}
			if proven {
				start, finish := s, s+task.Dwell
				check.Status = TaskVerified
				check.DwellStart = &start
				check.DwellFinish = &finish
				break
			}
		}
		if check.Status == "" {
			if len(missingOf(rep.Missing, a.VehicleID)) == 0 {
				check.Status = TaskIncomplete
			} else {
				check.Status = TaskUnknown
			}
		}
		rep.Tasks = append(rep.Tasks, check)
	}

	// Overall: any known violation or provably incomplete task fails;
	// otherwise missing observations leave the outcome unknown; full
	// coverage without problems passes.
	failed := len(rep.Violations) > 0
	for _, tc := range rep.Tasks {
		if tc.Status == TaskIncomplete {
			failed = true
		}
	}
	switch {
	case failed:
		rep.Status = VerifyFailed
	case len(rep.Missing) > 0:
		rep.Status = VerifyUnknown
	default:
		rep.Status = VerifyPassed
	}
	return rep
}

func missingOf(intervals []MissingInterval, vid string) []MissingInterval {
	out := []MissingInterval{}
	for _, m := range intervals {
		if m.VehicleID == vid {
			out = append(out, m)
		}
	}
	return out
}
