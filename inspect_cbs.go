package main

import (
	"container/heap"
	"context"
	"strings"
)

// iNode is one high-level CBS node for one task->vehicle assignment.
type iNode struct {
	assign []int            // task index -> vehicle index
	cons   []*constraintSet // per vehicle
	plans  []inspectPlan    // per vehicle
	cost   int
	seq    int
}

type iHeap struct {
	nodes []*iNode
}

func (h iHeap) Len() int { return len(h.nodes) }
func (h iHeap) Less(i, j int) bool {
	return h.nodes[i].cost < h.nodes[j].cost || (h.nodes[i].cost == h.nodes[j].cost && h.nodes[i].seq < h.nodes[j].seq)
}
func (h iHeap) Swap(i, j int) { h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i] }
func (h *iHeap) Push(x interface{}) {
	h.nodes = append(h.nodes, x.(*iNode))
}
func (h *iHeap) Pop() interface{} {
	old := h.nodes
	n := old[len(old)-1]
	h.nodes = old[:len(old)-1]
	return n
}

// enumerateAssignments returns every legal task->vehicle bijection
// (assign[task]=vehicle) honouring each task's allowed-vehicle set.
func enumerateAssignments(vehicles []VehicleSpec, tasks []TaskSpec) [][]int {
	n := len(tasks)
	allowed := make([]map[int]bool, n)
	vidx := map[string]int{}
	for i, v := range vehicles {
		vidx[v.ID] = i
	}
	for t, task := range tasks {
		allowed[t] = map[int]bool{}
		for _, id := range task.AllowedVehicles {
			allowed[t][vidx[id]] = true
		}
	}
	var out [][]int
	used := make([]bool, n)
	assign := make([]int, n)
	var rec func(t int)
	rec = func(t int) {
		if t == n {
			cp := make([]int, n)
			copy(cp, assign)
			out = append(out, cp)
			return
		}
		for v := 0; v < n; v++ {
			if !used[v] && allowed[t][v] {
				used[v] = true
				assign[t] = v
				rec(t + 1)
				used[v] = false
			}
		}
	}
	rec(0)
	return out
}

// rootInspect builds the unconstrained CBS root for one assignment.
func rootInspect(grid *Grid, vehicles []VehicleSpec, tasks []TaskSpec, assign []int, maxSteps int) (*iNode, bool) {
	n := len(vehicles)
	node := &iNode{
		assign: assign,
		cons:   make([]*constraintSet, n),
		plans:  make([]inspectPlan, n),
	}
	for ti, vi := range assign {
		node.cons[vi] = newConstraintSet()
		plan, ok := lowLevelInspect(grid, vehicles[vi], tasks[ti], node.cons[vi], maxSteps)
		if !ok {
			return nil, false
		}
		node.plans[vi] = plan
		node.cost += plan.arrival
	}
	return node, true
}

// inspectConflict finds the earliest vertex/edge conflict. Vehicles may
// share a cell with their own or other work cells; only vehicle-vehicle
// occupancy conflicts are considered.
func inspectConflict(plans []inspectPlan) *conflict {
	paths := make([][]Cell, len(plans))
	for i, p := range plans {
		paths[i] = p.path
	}
	return firstConflict(paths)
}

// iKey fingerprints assignment + joint constraints for pruning.
func iKey(nd *iNode) string {
	var sb strings.Builder
	for _, vi := range nd.assign {
		sb.WriteByte(byte('a' + vi))
	}
	sb.WriteByte('|')
	sb.WriteString(constraintKey(nd.cons))
	return sb.String()
}

// solveInspect jointly optimises task assignment and collision-free
// staged routes. All assignments share one CBS expansion budget.
func solveInspect(ctx context.Context, grid *Grid, vehicles []VehicleSpec, tasks []TaskSpec, maxSteps, budget int) ([]inspectPlan, []int, int, string) {
	assignments := enumerateAssignments(vehicles, tasks)
	if len(assignments) == 0 {
		return nil, nil, 0, StatusNoSolution
	}
	open := &iHeap{}
	heap.Init(open)
	seen := map[string]bool{}
	seq := 0
	for _, assign := range assignments {
		root, ok := rootInspect(grid, vehicles, tasks, assign, maxSteps)
		if !ok {
			continue // this assignment has no individually feasible route
		}
		root.seq = seq
		seq++
		heap.Push(open, root)
		seen[iKey(root)] = true
	}

	expanded := 0
	n := len(vehicles)
	for open.Len() > 0 {
		if err := ctx.Err(); err != nil {
			return nil, nil, 0, StatusUndecided
		}
		node := heap.Pop(open).(*iNode)
		conf := inspectConflict(node.plans)
		if conf == nil {
			return node.plans, node.assign, node.cost, StatusSuccess
		}
		if expanded >= budget {
			return nil, nil, 0, StatusUndecided
		}
		expanded++

		branches := [2]struct {
			agent int
			c     Constraint
		}{}
		if conf.edge {
			branches[0] = struct {
				agent int
				c     Constraint
			}{conf.a, Constraint{Agent: conf.a, Cell: conf.cell, From: conf.fromA, Time: conf.time - 1, Edge: true}}
			branches[1] = struct {
				agent int
				c     Constraint
			}{conf.b, Constraint{Agent: conf.b, Cell: conf.fromA, From: conf.fromB, Time: conf.time - 1, Edge: true}}
		} else {
			branches[0] = struct {
				agent int
				c     Constraint
			}{conf.a, Constraint{Agent: conf.a, Cell: conf.cell, Time: conf.time}}
			branches[1] = struct {
				agent int
				c     Constraint
			}{conf.b, Constraint{Agent: conf.b, Cell: conf.cell, Time: conf.time}}
		}

		for _, br := range branches {
			child := &iNode{
				assign: node.assign,
				cons:   make([]*constraintSet, n),
				plans:  make([]inspectPlan, n),
				seq:    seq,
			}
			seq++
			copy(child.cons, node.cons)
			copy(child.plans, node.plans)
			child.cost = node.cost
			child.cons[br.agent] = node.cons[br.agent].clone(br.c)
			ti := indexOfAssign(node.assign, br.agent)
			plan, ok := lowLevelInspect(grid, vehicles[br.agent], tasks[ti], child.cons[br.agent], maxSteps)
			if !ok {
				continue
			}
			child.cost += plan.arrival - child.plans[br.agent].arrival
			child.plans[br.agent] = plan
			key := iKey(child)
			if seen[key] {
				continue
			}
			seen[key] = true
			heap.Push(open, child)
		}
	}
	return nil, nil, 0, StatusNoSolution
}

// indexOfAssign returns the task index assigned to vehicle vi.
func indexOfAssign(assign []int, vi int) int {
	for ti, v := range assign {
		if v == vi {
			return ti
		}
	}
	return -1
}
