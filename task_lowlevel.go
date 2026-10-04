package main

import "container/heap"

// Route phases of one vehicle serving its assigned task.
const (
	phaseToJob  = iota // travelling to the job cell, may wait there
	phaseDwell         // dwelling at the job cell, uninterruptible
	phaseToGoal        // job done, travelling to the goal to park
)

// tlNode is an A* node over (cell, time, phase, dwell progress) states.
type tlNode struct {
	cell       Cell
	t          int
	phase      int
	k          int // dwell steps completed
	dwellStart int // time the dwell began (valid once phase > phaseToJob)
	g          int
	f          int
	prev       *tlNode
}

type tlHeap []*tlNode

func (h tlHeap) Len() int            { return len(h) }
func (h tlHeap) Less(i, j int) bool  { return h[i].f < h[j].f || (h[i].f == h[j].f && h[i].g > h[j].g) }
func (h tlHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *tlHeap) Push(x interface{}) { *h = append(*h, x.(*tlNode)) }
func (h *tlHeap) Pop() interface{} {
	old := *h
	n := old[len(old)-1]
	*h = old[:len(old)-1]
	return n
}

// lowLevelTask plans one vehicle's route for its assigned task:
// start -> job cell (waiting allowed) -> dwell task.Dwell consecutive
// steps inside [EarliestStart, LatestFinish] -> goal (park forever).
// Passing over the goal before the job is done does not park; parking
// is the first arrival at the goal in phaseToGoal. It returns the
// step-0..parking cells, the parking time and the dwell start/finish.
// ok is false when no route exists within maxSteps (this includes
// windows too short to hold the dwell).
func lowLevelTask(grid *Grid, start, goal Cell, task *TaskSpec, cs *constraintSet, maxSteps int) (path []Cell, parking, dwellStart, dwellFinish int, ok bool) {
	distJob := distField(grid, task.Cell)
	distGoal := distField(grid, goal)
	if _, reachable := distJob[start]; !reachable {
		return nil, 0, 0, 0, false
	}
	goalFromJob, reachable := distGoal[task.Cell]
	if !reachable {
		return nil, 0, 0, 0, false
	}
	if cs.vertexBlocked(start, 0) {
		return nil, 0, 0, 0, false
	}

	// heuristic returns an admissible lower bound on the remaining
	// time to parking, or -1 when the state cannot reach its target.
	heur := func(cell Cell, phase, k int) int {
		switch phase {
		case phaseToJob:
			d, ok := distJob[cell]
			if !ok {
				return -1
			}
			return d + task.Dwell + goalFromJob
		case phaseDwell:
			return task.Dwell - k + goalFromJob
		default:
			d, ok := distGoal[cell]
			if !ok {
				return -1
			}
			return d
		}
	}

	type state struct {
		cell  Cell
		t     int
		phase int
		k     int
	}
	best := map[state]int{{start, 0, phaseToJob, 0}: 0}
	open := &tlHeap{{cell: start, t: 0, phase: phaseToJob, g: 0, f: heur(start, phaseToJob, 0)}}
	heap.Init(open)

	for open.Len() > 0 {
		n := heap.Pop(open).(*tlNode)
		if n.g > best[state{n.cell, n.t, n.phase, n.k}] {
			continue
		}
		if n.phase == phaseToGoal && n.cell == goal && cs.futureSafe(goal, n.t) {
			path = make([]Cell, 0, n.t+1)
			for cur := n; cur != nil; cur = cur.prev {
				path = append(path, cur.cell)
			}
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path, n.t, n.dwellStart, n.dwellStart + task.Dwell, true
		}
		if n.t >= maxSteps {
			continue
		}
		nt := n.t + 1

		type move struct {
			to         Cell
			phase      int
			k          int
			dwellStart int
		}
		var moves []move
		switch n.phase {
		case phaseToJob:
			for _, d := range dirs {
				c := Cell{n.cell.X + d.X, n.cell.Y + d.Y}
				if grid.free(c) {
					moves = append(moves, move{to: c, phase: phaseToJob})
				}
			}
			moves = append(moves, move{to: n.cell, phase: phaseToJob}) // wait in place
			if n.cell == task.Cell && n.t >= task.EarliestStart && n.t+task.Dwell <= task.LatestFinish {
				// Begin the dwell: the first of Dwell consecutive stays.
				// When Dwell == 1 the dwell completes with this stay.
				phase := phaseDwell
				if task.Dwell == 1 {
					phase = phaseToGoal
				}
				moves = append(moves, move{to: n.cell, phase: phase, k: 1, dwellStart: n.t})
			}
		case phaseDwell:
			// Dwelling is uninterruptible: only staying at the job cell.
			if n.k+1 == task.Dwell {
				moves = append(moves, move{to: n.cell, phase: phaseToGoal, k: n.k + 1, dwellStart: n.dwellStart})
			} else {
				moves = append(moves, move{to: n.cell, phase: phaseDwell, k: n.k + 1, dwellStart: n.dwellStart})
			}
		default: // phaseToGoal
			for _, d := range dirs {
				c := Cell{n.cell.X + d.X, n.cell.Y + d.Y}
				if grid.free(c) {
					moves = append(moves, move{to: c, phase: phaseToGoal, dwellStart: n.dwellStart})
				}
			}
			moves = append(moves, move{to: n.cell, phase: phaseToGoal, dwellStart: n.dwellStart})
		}

		for _, m := range moves {
			if cs.vertexBlocked(m.to, nt) || cs.edgeBlocked(n.cell, m.to, n.t) {
				continue
			}
			h := heur(m.to, m.phase, m.k)
			if h < 0 {
				continue
			}
			ng := n.g + 1
			s := state{m.to, nt, m.phase, m.k}
			if prev, seen := best[s]; seen && prev <= ng {
				continue
			}
			best[s] = ng
			heap.Push(open, &tlNode{
				cell: m.to, t: nt, phase: m.phase, k: m.k,
				dwellStart: m.dwellStart, g: ng, f: ng + h, prev: n,
			})
		}
	}
	return nil, 0, 0, 0, false
}
