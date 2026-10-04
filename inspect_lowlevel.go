package main

import "container/heap"

// Inspection phases:
//   phase 0: before the uninterrupted dwell starts (may wait / pass the goal)
//   phase 1: dwelling on the work cell, d dwell steps performed so far
//   phase 2: dwell complete, heading to the permanent goal

type illState struct {
	cell  Cell
	t     int
	phase int
	d     int
}

type illNode struct {
	state     illState
	g, f      int
	servStart int // dwell start time; meaningful in phases 1 and 2
	prev      *illNode
}

type illHeap []*illNode

func (h illHeap) Len() int            { return len(h) }
func (h illHeap) Less(i, j int) bool  { return h[i].f < h[j].f || (h[i].f == h[j].f && h[i].g > h[j].g) }
func (h illHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *illHeap) Push(x interface{}) { *h = append(*h, x.(*illNode)) }
func (h *illHeap) Pop() interface{} {
	old := *h
	n := old[len(old)-1]
	*h = old[:len(old)-1]
	return n
}

// inspectPlan is the single-vehicle result of lowLevelInspect.
type inspectPlan struct {
	path               []Cell
	arrival            int
	servStart, servFin int
}

// lowLevelInspect plans one vehicle: start -> (wait) -> uninterrupted
// dwell on task.Work -> permanent stay at goal after the first post-dwell
// arrival. The dwell occupies times s..s+dwell-1 and must satisfy the
// task window. ok is false if no such plan exists within maxSteps.
func lowLevelInspect(grid *Grid, v VehicleSpec, task TaskSpec, cs *constraintSet, maxSteps int) (inspectPlan, bool) {
	distWork := distField(grid, task.Work)
	distGoal := distField(grid, v.Goal)
	if _, reach := distWork[v.Start]; !reach {
		return inspectPlan{}, false
	}
	if _, reach := distGoal[task.Work]; !reach {
		return inspectPlan{}, false
	}
	if cs.vertexBlocked(v.Start, 0) {
		return inspectPlan{}, false
	}

	startState := illState{cell: v.Start, t: 0, phase: 0}
	best := map[illState]int{startState: 0}
	open := &illHeap{{state: startState, g: 0, f: distWork[v.Start]}}
	heap.Init(open)

	neighbors := func(c Cell) []Cell {
		out := make([]Cell, 0, 5)
		for _, d := range dirs {
			n := Cell{c.X + d.X, c.Y + d.Y}
			if grid.free(n) {
				out = append(out, n)
			}
		}
		return append(out, c)
	}
	build := func(n *illNode) inspectPlan {
		out := make([]Cell, 0, n.state.t+1)
		for cur := n; cur != nil; cur = cur.prev {
			out = append(out, cur.state.cell)
		}
		for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
			out[i], out[j] = out[j], out[i]
		}
		return inspectPlan{
			path:      out,
			arrival:   n.state.t,
			servStart: n.servStart,
			servFin:   n.servStart + task.DwellSteps - 1,
		}
	}
	push := func(prev *illNode, st illState, h, serv int) {
		ng := prev.g + 1
		if old, seen := best[st]; seen && old <= ng {
			return
		}
		best[st] = ng
		heap.Push(open, &illNode{state: st, g: ng, f: ng + h, servStart: serv, prev: prev})
	}
	arriveGoal := func(prev *illNode, t int) (inspectPlan, bool) {
		if !cs.futureSafe(v.Goal, t) {
			return inspectPlan{}, false
		}
		n := &illNode{state: illState{cell: v.Goal, t: t, phase: 2}, g: prev.g + 1, servStart: prev.servStart, prev: prev}
		return build(n), true
	}

	for open.Len() > 0 {
		n := heap.Pop(open).(*illNode)
		if n.g > best[n.state] {
			continue
		}
		// After the dwell, first arrival on the goal is a permanent stop.
		if n.state.phase == 2 && n.state.cell == v.Goal && cs.futureSafe(v.Goal, n.state.t) {
			return build(n), true
		}
		if n.state.t >= maxSteps {
			continue
		}
		st := n.state
		nt := st.t + 1

		switch st.phase {
		case 0:
			for _, to := range neighbors(st.cell) {
				if cs.vertexBlocked(to, nt) || cs.edgeBlocked(st.cell, to, st.t) {
					continue
				}
				canDwell := to == task.Work &&
					nt >= task.EarliestStart && nt <= task.LatestFinish-task.DwellSteps+1
				if canDwell {
					nh := task.DwellSteps - 1 + distGoal[task.Work]
					push(n, illState{cell: task.Work, t: nt, phase: 1, d: 1}, nh, nt)
				}
				// Passing the goal (or the work cell while waiting) before
				// the dwell is travel, never parking.
				if !canDwell {
					push(n, illState{cell: to, t: nt, phase: 0}, distWork[to], 0)
				}
			}
		case 1:
			// Uninterrupted in-place dwell on Work.
			if cs.vertexBlocked(task.Work, nt) || cs.edgeBlocked(task.Work, task.Work, st.t) {
				continue
			}
			if st.d+1 < task.DwellSteps {
				push(n, illState{cell: task.Work, t: nt, phase: 1, d: st.d + 1},
					task.DwellSteps-st.d-1+distGoal[task.Work], n.servStart)
			} else {
				// The step to nt completes the dwell. Enter phase 2 at the
				// same time; when Work is the goal this is already a stop
				// (handled when popped), otherwise the next step moves off.
				push(n, illState{cell: task.Work, t: nt, phase: 2}, distGoal[task.Work], n.servStart)
			}
		case 2:
			for _, to := range neighbors(st.cell) {
				if cs.vertexBlocked(to, nt) || cs.edgeBlocked(st.cell, to, st.t) {
					continue
				}
				if to == v.Goal {
					if plan, ok := arriveGoal(n, nt); ok {
						return plan, true
					}
					continue
				}
				// Once dwell is complete, waiting at Work is allowed while
				// routing to the goal.
				push(n, illState{cell: to, t: nt, phase: 2}, distGoal[to], n.servStart)
			}
		}
	}
	return inspectPlan{}, false
}
