package main

import (
	"container/heap"
	"context"
	"fmt"
	"strings"
)

// taskNode is one CBS high-level node for one concrete assignment.
type taskNode struct {
	assign      []int // vehicle i -> task index
	constraints []*constraintSet
	paths       [][]Cell
	arrivals    []int // parking times
	dwellStart  []int
	cost        int // sum of parking times
	seq         int
}

type taskHeap []taskNode

func (h taskHeap) Len() int { return len(h) }
func (h taskHeap) Less(i, j int) bool {
	return h[i].cost < h[j].cost || (h[i].cost == h[j].cost && h[i].seq < h[j].seq)
}
func (h taskHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *taskHeap) Push(x interface{}) { *h = append(*h, x.(taskNode)) }
func (h *taskHeap) Pop() interface{} {
	old := *h
	n := old[len(old)-1]
	*h = old[:len(old)-1]
	return n
}

// assignments enumerates every bijection vehicle i -> task perm[i]
// that respects each task's executable vehicle set.
func assignments(vehicles []VehicleSpec, tasks []TaskSpec) [][]int {
	executable := make([]map[string]bool, len(tasks))
	for j, task := range tasks {
		executable[j] = map[string]bool{}
		for _, vid := range task.Vehicles {
			executable[j][vid] = true
		}
	}
	var out [][]int
	perm := make([]int, len(vehicles))
	used := make([]bool, len(tasks))
	var walk func(i int)
	walk = func(i int) {
		if i == len(vehicles) {
			cp := make([]int, len(perm))
			copy(cp, perm)
			out = append(out, cp)
			return
		}
		for j := range tasks {
			if used[j] || !executable[j][vehicles[i].ID] {
				continue
			}
			used[j] = true
			perm[i] = j
			walk(i + 1)
			used[j] = false
		}
	}
	walk(0)
	return out
}

// solveInspect jointly optimizes task assignment and collision-free
// routes. Every legal one-to-one assignment gets its own CBS tree, but
// all trees share one priority queue ordered by total cost and one
// expansion budget, so the first conflict-free node popped is the
// proven global optimum across all assignments. Status semantics:
//   - StatusSuccess:    optimal joint plan found and proven.
//   - StatusNoSolution: every assignment's search space exhausted.
//   - StatusUndecided:  shared budget ran out, or ctx was cancelled.
func solveInspect(ctx context.Context, grid *Grid, vehicles []VehicleSpec, tasks []TaskSpec, maxSteps, budget int) (assign []int, paths [][]Cell, arrivals, dwellStarts []int, cost int, status string) {
	n := len(vehicles)
	open := &taskHeap{}
	heap.Init(open)
	seq := 0
	seen := map[string]bool{}

	push := func(node taskNode) {
		key := taskNodeKey(node.assign, node.constraints)
		if seen[key] {
			return
		}
		seen[key] = true
		node.seq = seq
		seq++
		heap.Push(open, node)
	}

	// One root per legal assignment; assignments whose root is already
	// infeasible (unreachable job/goal, window too short for the dwell)
	// simply contribute nothing to the shared search.
	for _, perm := range assignments(vehicles, tasks) {
		root := taskNode{
			assign:      perm,
			constraints: make([]*constraintSet, n),
			paths:       make([][]Cell, n),
			arrivals:    make([]int, n),
			dwellStart:  make([]int, n),
		}
		feasible := true
		for i, v := range vehicles {
			root.constraints[i] = newConstraintSet()
			p, arr, ds, _, ok := lowLevelTask(grid, v.Start, v.Goal, &tasks[perm[i]], root.constraints[i], maxSteps)
			if !ok {
				feasible = false
				break
			}
			root.paths[i] = p
			root.arrivals[i] = arr
			root.dwellStart[i] = ds
			root.cost += arr
		}
		if feasible {
			push(root)
		}
	}

	expanded := 0
	for open.Len() > 0 {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, nil, 0, StatusUndecided
		}
		node := heap.Pop(open).(taskNode)
		conf := firstConflict(node.paths)
		if conf == nil {
			return node.assign, node.paths, node.arrivals, node.dwellStart, node.cost, StatusSuccess
		}
		if expanded >= budget {
			return nil, nil, nil, nil, 0, StatusUndecided
		}
		expanded++

		branch := []struct {
			agent int
			c     Constraint
		}{}
		if conf.edge {
			branch = append(branch,
				struct {
					agent int
					c     Constraint
				}{conf.a, Constraint{Agent: conf.a, Cell: conf.cell, From: conf.fromA, Time: conf.time - 1, Edge: true}},
				struct {
					agent int
					c     Constraint
				}{conf.b, Constraint{Agent: conf.b, Cell: conf.fromA, From: conf.fromB, Time: conf.time - 1, Edge: true}},
			)
		} else {
			branch = append(branch,
				struct {
					agent int
					c     Constraint
				}{conf.a, Constraint{Agent: conf.a, Cell: conf.cell, Time: conf.time}},
				struct {
					agent int
					c     Constraint
				}{conf.b, Constraint{Agent: conf.b, Cell: conf.cell, Time: conf.time}},
			)
		}

		for _, br := range branch {
			child := taskNode{
				assign:      node.assign,
				constraints: make([]*constraintSet, n),
				paths:       make([][]Cell, n),
				arrivals:    make([]int, n),
				dwellStart:  make([]int, n),
			}
			copy(child.constraints, node.constraints)
			copy(child.paths, node.paths)
			copy(child.arrivals, node.arrivals)
			copy(child.dwellStart, node.dwellStart)
			child.cost = node.cost
			child.constraints[br.agent] = node.constraints[br.agent].clone(br.c)
			v := vehicles[br.agent]
			p, arr, ds, _, ok := lowLevelTask(grid, v.Start, v.Goal, &tasks[child.assign[br.agent]], child.constraints[br.agent], maxSteps)
			if !ok {
				continue
			}
			child.paths[br.agent] = p
			child.cost += arr - child.arrivals[br.agent]
			child.arrivals[br.agent] = arr
			child.dwellStart[br.agent] = ds
			push(child)
		}
	}
	return nil, nil, nil, nil, 0, StatusNoSolution
}

// taskNodeKey fingerprints assignment plus joint constraint state.
func taskNodeKey(assign []int, sets []*constraintSet) string {
	var sb strings.Builder
	for _, j := range assign {
		fmt.Fprintf(&sb, "%d,", j)
	}
	sb.WriteString(constraintKey(sets))
	return sb.String()
}
