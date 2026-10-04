package main

import (
	"context"
	"testing"
)

func inspectReq() *InspectRequest {
	return &InspectRequest{
		Width: 7, Height: 3, MaxSteps: 40, NodeBudget: 20000,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{6, 0}},
			{ID: "b", Start: Cell{0, 2}, Goal: Cell{6, 2}},
		},
		Tasks: []TaskSpec{
			{ID: "t1", Work: Cell{2, 0}, AllowedVehicles: []string{"a", "b"},
				EarliestStart: 2, LatestFinish: 12, DwellSteps: 2},
			{ID: "t2", Work: Cell{4, 2}, AllowedVehicles: []string{"a", "b"},
				EarliestStart: 0, LatestFinish: 20, DwellSteps: 1},
		},
	}
}

func solveInspectReq(t *testing.T, req *InspectRequest) ([]inspectPlan, []int, int, string) {
	t.Helper()
	grid, vehicles, tasks, err := validateInspectRequest(req)
	if err != nil {
		t.Fatalf("validate: %v", err)
	}
	return solveInspect(context.Background(), grid, vehicles, tasks, req.MaxSteps, req.NodeBudget)
}

// checkInspect simulates the joint plan: movement rules, dwell windows,
// uninterrupted dwell and inter-vehicle conflicts.
func checkInspect(t *testing.T, req *InspectRequest, plans []inspectPlan, assign []int) {
	t.Helper()
	grid, _, _, _ := validateInspectRequest(req)
	n := len(req.Vehicles)
	for ti, vi := range assign {
		task := req.Tasks[ti]
		p := plans[vi]
		if p.path[0] != req.Vehicles[vi].Start {
			t.Errorf("vehicle %s bad start", req.Vehicles[vi].ID)
		}
		if p.path[p.arrival] != req.Vehicles[vi].Goal {
			t.Errorf("vehicle %s bad parked goal", req.Vehicles[vi].ID)
		}
		if p.arrival > req.MaxSteps {
			t.Errorf("vehicle %s exceeds max steps", req.Vehicles[vi].ID)
		}
		if p.servStart < task.EarliestStart || p.servFin > task.LatestFinish {
			t.Errorf("vehicle %s dwell outside window: %d..%d vs %d..%d",
				req.Vehicles[vi].ID, p.servStart, p.servFin, task.EarliestStart, task.LatestFinish)
		}
		if p.servFin-p.servStart+1 != task.DwellSteps {
			t.Errorf("vehicle %s dwell length wrong", req.Vehicles[vi].ID)
		}
		for k := p.servStart; k <= p.servFin; k++ {
			if p.path[k] != task.Work {
				t.Errorf("vehicle %s interrupts dwell at %d", req.Vehicles[vi].ID, k)
			}
		}
		for k := 1; k <= p.arrival; k++ {
			d := abs(p.path[k].X-p.path[k-1].X) + abs(p.path[k].Y-p.path[k-1].Y)
			if d > 1 {
				t.Errorf("vehicle %s teleports", req.Vehicles[vi].ID)
			}
			if !grid.free(p.path[k]) {
				t.Errorf("vehicle %s enters blocked cell", req.Vehicles[vi].ID)
			}
		}
		// After dwell, the first time the goal is reached must be arrival.
		for k := p.servFin + 1; k < p.arrival; k++ {
			if p.path[k] == req.Vehicles[vi].Goal {
				t.Errorf("vehicle %s visited goal before parking", req.Vehicles[vi].ID)
			}
		}
	}
	horizon := req.MaxSteps
	paths := make([][]Cell, n)
	for i, p := range plans {
		paths[i] = p.path
	}
	for ts := 0; ts <= horizon; ts++ {
		for i := 0; i < n; i++ {
			for j := i + 1; j < n; j++ {
				if posAt(paths[i], ts) == posAt(paths[j], ts) {
					t.Fatalf("vertex conflict at t=%d", ts)
				}
				if ts > 0 {
					a0, a1 := posAt(paths[i], ts-1), posAt(paths[i], ts)
					b0, b1 := posAt(paths[j], ts-1), posAt(paths[j], ts)
					if a0 == b1 && b0 == a1 && a0 != a1 {
						t.Fatalf("edge swap at t=%d", ts)
					}
				}
			}
		}
	}
}

func TestInspectSingleVehicle(t *testing.T) {
	req := &InspectRequest{
		Width: 5, Height: 1, MaxSteps: 20, NodeBudget: 100,
		Vehicles: []VehicleSpec{{ID: "a", Start: Cell{0, 0}, Goal: Cell{4, 0}}},
		Tasks: []TaskSpec{{ID: "j", Work: Cell{2, 0}, AllowedVehicles: []string{"a"},
			EarliestStart: 3, LatestFinish: 8, DwellSteps: 2}},
	}
	plans, assign, cost, status := solveInspectReq(t, req)
	if status != StatusSuccess {
		t.Fatalf("status=%s", status)
	}
	// wait at (1,0) one step, dwell 3..4, then goal at 6.
	p := plans[assign[0]]
	if p.servStart != 3 || p.servFin != 4 || p.arrival != 6 || cost != 6 {
		t.Fatalf("plan=%+v", p)
	}
	checkInspect(t, req, plans, assign)
}

func TestInspectTightWindowStillFeasible(t *testing.T) {
	// Earliest dwell at t=1..2, finish exactly at the window end.
	req := &InspectRequest{
		Width: 3, Height: 1, MaxSteps: 10, NodeBudget: 100,
		Vehicles: []VehicleSpec{{ID: "a", Start: Cell{0, 0}, Goal: Cell{2, 0}}},
		Tasks: []TaskSpec{{ID: "j", Work: Cell{1, 0}, AllowedVehicles: []string{"a"},
			EarliestStart: 1, LatestFinish: 2, DwellSteps: 2}},
	}
	_, _, _, status := solveInspectReq(t, req)
	plans, assign, _, status := solveInspectReq(t, req)
	if status != StatusSuccess {
		t.Fatalf("status=%s", status)
	}
	p := plans[assign[0]]
	if p.servStart != 1 || p.servFin != 2 || p.arrival != 3 {
		t.Fatalf("plan=%+v", p)
	}
}

func TestInspectWindowCannotHoldDwellIsNoSolution(t *testing.T) {
	req := &InspectRequest{
		Width: 5, Height: 1, MaxSteps: 20, NodeBudget: 100,
		Vehicles: []VehicleSpec{{ID: "a", Start: Cell{0, 0}, Goal: Cell{4, 0}}},
		Tasks: []TaskSpec{{ID: "j", Work: Cell{2, 0}, AllowedVehicles: []string{"a"},
			EarliestStart: 3, LatestFinish: 3, DwellSteps: 2}},
	}
	_, _, _, status := solveInspectReq(t, req)
	if status != StatusNoSolution {
		t.Fatalf("status=%s, want no_solution", status)
	}
}

func TestInspectAssignmentOptimalAcrossBijection(t *testing.T) {
	req := inspectReq()
	plans, assign, cost, status := solveInspectReq(t, req)
	if status != StatusSuccess {
		t.Fatalf("status=%s", status)
	}
	checkInspect(t, req, plans, assign)
	if assign[0] != 0 || assign[1] != 1 {
		t.Fatalf("expected natural assignment, got %v", assign)
	}
	if cost <= 0 {
		t.Fatalf("bad cost %d", cost)
	}
}

// A task whose allowed set excludes the only nearby vehicle forces a
// specific bijection.
func TestInspectForcedAssignment(t *testing.T) {
	req := inspectReq()
	req.Tasks[0].AllowedVehicles = []string{"b"}
	req.Tasks[1].AllowedVehicles = []string{"a"}
	plans, assign, _, status := solveInspectReq(t, req)
	if status != StatusSuccess {
		t.Fatalf("status=%s", status)
	}
	if assign[0] != 1 || assign[1] != 0 {
		t.Fatalf("got %v", assign)
	}
	checkInspect(t, req, plans, assign)
}

func TestInspectNoFeasibleBijection(t *testing.T) {
	req := inspectReq()
	req.Tasks[0].AllowedVehicles = []string{"a"}
	req.Tasks[1].AllowedVehicles = []string{"a"}
	_, _, _, status := solveInspectReq(t, req)
	if status != StatusNoSolution {
		t.Fatalf("status=%s, want no_solution", status)
	}
}

func TestInspectWindowTooTight(t *testing.T) {
	req := inspectReq()
	// Work at x=2 needs at least t=2 to reach; a 2-step dwell cannot fit
	// a finish deadline of 2 (b starts even farther away).
	req.Tasks[0].EarliestStart = 0
	req.Tasks[0].LatestFinish = 2
	req.Tasks[0].DwellSteps = 2
	_, _, _, status := solveInspectReq(t, req)
	if status != StatusNoSolution {
		t.Fatalf("status=%s, want no_solution", status)
	}
}

func TestInspectUndecidedOnBudget(t *testing.T) {
	req := inspectReq()
	req.NodeBudget = 1 // budget one: root conflict cannot be resolved
	req.Width, req.Height = 3, 2
	req.Vehicles = []VehicleSpec{
		{ID: "a", Start: Cell{0, 0}, Goal: Cell{2, 0}},
		{ID: "b", Start: Cell{2, 0}, Goal: Cell{0, 0}},
	}
	req.Tasks = []TaskSpec{
		{ID: "t1", Work: Cell{0, 1}, AllowedVehicles: []string{"a", "b"}, EarliestStart: 0, LatestFinish: 16, DwellSteps: 2},
		{ID: "t2", Work: Cell{2, 1}, AllowedVehicles: []string{"a", "b"}, EarliestStart: 0, LatestFinish: 16, DwellSteps: 2},
	}
	grid, vehicles, tasks, err := validateInspectRequest(req)
	if err != nil {
		t.Fatal(err)
	}
	_, _, _, status := solveInspect(context.Background(), grid, vehicles, tasks, req.MaxSteps, req.NodeBudget)
	if status != StatusUndecided {
		t.Fatalf("status=%s, want undecided", status)
	}
}

func TestInspectValidateRejects(t *testing.T) {
	cases := map[string]func(*InspectRequest){
		"four cars": func(r *InspectRequest) {
			r.Vehicles = append(r.Vehicles,
				VehicleSpec{ID: "c", Start: Cell{0, 1}, Goal: Cell{6, 1}},
				VehicleSpec{ID: "d", Start: Cell{1, 1}, Goal: Cell{5, 1}})
		},
		"task count mismatch": func(r *InspectRequest) { r.Tasks = r.Tasks[:1] },
		"dup task id":         func(r *InspectRequest) { r.Tasks[1].ID = "t1" },
		"work on obstacle":    func(r *InspectRequest) { r.Obstacles = []Cell{{2, 0}} },
		"work oob":            func(r *InspectRequest) { r.Tasks[0].Work = Cell{9, 9} },
		"bad time":            func(r *InspectRequest) { r.Tasks[0].LatestFinish = 65 },
		"negative time":       func(r *InspectRequest) { r.Tasks[0].EarliestStart = -1 },
		"inverted window":     func(r *InspectRequest) { r.Tasks[0].EarliestStart = 10; r.Tasks[0].LatestFinish = 3 },
		"dwell zero":          func(r *InspectRequest) { r.Tasks[0].DwellSteps = 0 },
		"dwell nine":          func(r *InspectRequest) { r.Tasks[0].DwellSteps = 9 },
		"empty allowed":       func(r *InspectRequest) { r.Tasks[0].AllowedVehicles = nil },
		"unknown vehicle":     func(r *InspectRequest) { r.Tasks[0].AllowedVehicles = []string{"zz"} },
		"allowed duplicate":   func(r *InspectRequest) { r.Tasks[0].AllowedVehicles = []string{"a", "a"} },
	}
	for name, mutate := range cases {
		req := inspectReq()
		mutate(req)
		if _, _, _, err := validateInspectRequest(req); err == nil {
			t.Errorf("%s: expected error", name)
		}
	}
}

func TestStartEqualsGoalStaysPut(t *testing.T) {
	req := &PlanRequest{
		Width: 3, Height: 1, MaxSteps: 12, NodeBudget: 20000,
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{1, 0}, Goal: Cell{1, 0}},
			{ID: "b", Start: Cell{0, 0}, Goal: Cell{2, 0}},
		},
	}
	grid, _ := validateRequest(req)
	paths, _, _, status := solveCBS(context.Background(), grid, req.Vehicles, req.MaxSteps, req.NodeBudget)
	if status != StatusNoSolution {
		t.Fatalf("status=%s, want no_solution", status)
	}
	_ = paths
}
