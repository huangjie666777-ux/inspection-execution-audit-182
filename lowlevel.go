package main

import "container/heap"

// Constraint forbids one vehicle from occupying a cell at a time,
// or from traversing an edge between two consecutive times.
type Constraint struct {
	Agent int
	Cell  Cell
	From  Cell // only meaningful when Edge is true
	Time  int
	Edge  bool
}

type constraintSet struct {
	vertex map[Cell]map[int]bool
	edge   map[[2]Cell]map[int]bool
	maxT   int
}

func newConstraintSet() *constraintSet {
	return &constraintSet{
		vertex: map[Cell]map[int]bool{},
		edge:   map[[2]Cell]map[int]bool{},
	}
}

func (cs *constraintSet) add(c Constraint) {
	if c.Edge {
		k := [2]Cell{c.From, c.Cell}
		if cs.edge[k] == nil {
			cs.edge[k] = map[int]bool{}
		}
		cs.edge[k][c.Time] = true
	} else {
		if cs.vertex[c.Cell] == nil {
			cs.vertex[c.Cell] = map[int]bool{}
		}
		cs.vertex[c.Cell][c.Time] = true
	}
	if c.Time > cs.maxT {
		cs.maxT = c.Time
	}
}

func (cs *constraintSet) vertexBlocked(c Cell, t int) bool {
	return cs.vertex[c][t]
}

func (cs *constraintSet) edgeBlocked(from, to Cell, t int) bool {
	return cs.edge[[2]Cell{from, to}][t]
}

// clone returns an independent copy with one extra constraint added.
func (cs *constraintSet) clone(c Constraint) *constraintSet {
	n := newConstraintSet()
	n.maxT = cs.maxT
	for cell, ts := range cs.vertex {
		n.vertex[cell] = map[int]bool{}
		for t := range ts {
			n.vertex[cell][t] = true
		}
	}
	for k, ts := range cs.edge {
		n.edge[k] = map[int]bool{}
		for t := range ts {
			n.edge[k][t] = true
		}
	}
	n.add(c)
	return n
}

// futureSafe reports whether the agent may rest at goal forever from
// time t on, i.e. no vertex constraint touches goal at any time >= t.
func (cs *constraintSet) futureSafe(goal Cell, t int) bool {
	ts := cs.vertex[goal]
	for ct := range ts {
		if ct >= t {
			return false
		}
	}
	return true
}

// A* node over (cell, time) states.
type llNode struct {
	cell Cell
	t    int
	g    int
	f    int
	prev *llNode
}

type llHeap []*llNode

func (h llHeap) Len() int            { return len(h) }
func (h llHeap) Less(i, j int) bool  { return h[i].f < h[j].f || (h[i].f == h[j].f && h[i].g > h[j].g) }
func (h llHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *llHeap) Push(x interface{}) { *h = append(*h, x.(*llNode)) }
func (h *llHeap) Pop() interface{} {
	old := *h
	n := old[len(old)-1]
	*h = old[:len(old)-1]
	return n
}

// distField runs BFS from goal over free cells; dist[c] is the
// obstacle-aware Manhattan distance to goal, -1 if unreachable.
func distField(grid *Grid, goal Cell) map[Cell]int {
	dist := map[Cell]int{goal: 0}
	queue := []Cell{goal}
	for len(queue) > 0 {
		c := queue[0]
		queue = queue[1:]
		for _, d := range dirs {
			n := Cell{c.X + d.X, c.Y + d.Y}
			if grid.free(n) {
				if _, ok := dist[n]; !ok {
					dist[n] = dist[c] + 1
					queue = append(queue, n)
				}
			}
		}
	}
	return dist
}

// lowLevel plans a single-vehicle path from start to goal honouring the
// constraints. It returns the step-0..arrival cells and the arrival time.
// ok is false when no plan exists within maxSteps.
func lowLevel(grid *Grid, start, goal Cell, cs *constraintSet, maxSteps int) (path []Cell, arrival int, ok bool) {
	dist := distField(grid, goal)
	h0, reachable := dist[start]
	if !reachable {
		return nil, 0, false
	}
	if cs.vertexBlocked(start, 0) {
		return nil, 0, false
	}
	// A vehicle already on its goal parks from time 0; it never drives
	// away and comes back. If a constraint touches the goal at any
	// time, the branch is infeasible for this vehicle.
	if start == goal {
		if cs.futureSafe(goal, 0) {
			return []Cell{start}, 0, true
		}
		return nil, 0, false
	}

	type state struct {
		cell Cell
		t    int
	}
	best := map[state]int{{start, 0}: 0}
	open := &llHeap{{cell: start, t: 0, g: 0, f: h0}}
	heap.Init(open)

	for open.Len() > 0 {
		n := heap.Pop(open).(*llNode)
		if n.g > best[state{n.cell, n.t}] {
			continue
		}
		if n.cell == goal && cs.futureSafe(goal, n.t) {
			path = make([]Cell, 0, n.t+1)
			for cur := n; cur != nil; cur = cur.prev {
				path = append(path, cur.cell)
			}
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path, n.t, true
		}
		if n.t >= maxSteps {
			continue
		}
		nt := n.t + 1
		moves := make([]Cell, 0, 5)
		for _, d := range dirs {
			c := Cell{n.cell.X + d.X, n.cell.Y + d.Y}
			if grid.free(c) {
				moves = append(moves, c)
			}
		}
		moves = append(moves, n.cell) // wait in place
		for _, to := range moves {
			if cs.vertexBlocked(to, nt) || cs.edgeBlocked(n.cell, to, n.t) {
				continue
			}
			ng := n.g + 1
			s := state{to, nt}
			if prev, seen := best[s]; seen && prev <= ng {
				continue
			}
			best[s] = ng
			h := dist[to]
			heap.Push(open, &llNode{cell: to, t: nt, g: ng, f: ng + h, prev: n})
		}
	}
	return nil, 0, false
}
