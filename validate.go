package main

import (
	"errors"
	"fmt"
)

// validateRequest checks all request-level invariants and returns a Grid.
func validateRequest(req *PlanRequest) (*Grid, error) {
	grid, err := validateBase(req.Width, req.Height, req.MaxSteps, req.NodeBudget, req.Obstacles)
	if err != nil {
		return nil, err
	}
	if len(req.Vehicles) < 1 || len(req.Vehicles) > maxVehicles {
		return nil, fmt.Errorf("vehicles must contain 1..%d entries", maxVehicles)
	}
	if err := validateVehicles(grid, req.Vehicles); err != nil {
		return nil, err
	}
	return grid, nil
}

// validateBase checks the shared map / step / budget invariants.
func validateBase(width, height, maxSteps, nodeBudget int, obstacles []Cell) (*Grid, error) {
	if width < 1 || width > maxGridDim || height < 1 || height > maxGridDim {
		return nil, fmt.Errorf("width and height must be within 1..%d", maxGridDim)
	}
	if maxSteps < 1 || maxSteps > maxStepsLimit {
		return nil, fmt.Errorf("max_steps must be within 1..%d", maxStepsLimit)
	}
	if nodeBudget < 1 || nodeBudget > maxBudget {
		return nil, fmt.Errorf("node_budget must be within 1..%d", maxBudget)
	}
	grid := &Grid{W: width, H: height, Blocked: map[Cell]bool{}}
	for _, o := range obstacles {
		if o.X < 0 || o.X >= width || o.Y < 0 || o.Y >= height {
			return nil, fmt.Errorf("obstacle (%d,%d) out of bounds", o.X, o.Y)
		}
		grid.Blocked[o] = true
	}
	return grid, nil
}

// validateVehicles checks vehicle ids, starts and goals.
func validateVehicles(grid *Grid, vehicles []VehicleSpec) error {
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

// validateInspectRequest checks every invariant of POST /inspect and
// returns the grid together with the validated vehicles and tasks.
func validateInspectRequest(req *InspectRequest) (*Grid, []VehicleSpec, []TaskSpec, error) {
	grid, err := validateBase(req.Width, req.Height, req.MaxSteps, req.NodeBudget, req.Obstacles)
	if err != nil {
		return nil, nil, nil, err
	}
	if len(req.Vehicles) < 1 || len(req.Vehicles) > maxInspectVehicles {
		return nil, nil, nil, fmt.Errorf("vehicles must contain 1..%d entries", maxInspectVehicles)
	}
	if err := validateVehicles(grid, req.Vehicles); err != nil {
		return nil, nil, nil, err
	}
	if len(req.Tasks) != len(req.Vehicles) {
		return nil, nil, nil, fmt.Errorf("tasks must contain exactly %d entries (one per vehicle)", len(req.Vehicles))
	}

	vehicleIDs := map[string]bool{}
	for _, v := range req.Vehicles {
		vehicleIDs[v.ID] = true
	}
	taskIDs := map[string]bool{}
	tasks := make([]TaskSpec, len(req.Tasks))
	for i, task := range req.Tasks {
		if task.ID == "" {
			return nil, nil, nil, errors.New("task id must not be empty")
		}
		if taskIDs[task.ID] {
			return nil, nil, nil, fmt.Errorf("duplicate task id %q", task.ID)
		}
		taskIDs[task.ID] = true
		if !grid.free(task.Work) {
			return nil, nil, nil, fmt.Errorf("task %q work cell (%d,%d) out of bounds or on obstacle", task.ID, task.Work.X, task.Work.Y)
		}
		if task.EarliestStart < 0 || task.EarliestStart > maxTaskTime {
			return nil, nil, nil, fmt.Errorf("task %q earliest_start must be within 0..%d", task.ID, maxTaskTime)
		}
		if task.LatestFinish < 0 || task.LatestFinish > maxTaskTime {
			return nil, nil, nil, fmt.Errorf("task %q latest_finish must be within 0..%d", task.ID, maxTaskTime)
		}
		if task.EarliestStart > task.LatestFinish {
			return nil, nil, nil, fmt.Errorf("task %q time window is inverted", task.ID)
		}
		if task.DwellSteps < 1 || task.DwellSteps > maxDwellSteps {
			return nil, nil, nil, fmt.Errorf("task %q dwell_steps must be within 1..%d", task.ID, maxDwellSteps)
		}
		if len(task.AllowedVehicles) == 0 {
			return nil, nil, nil, fmt.Errorf("task %q allowed_vehicles must be a non-empty list", task.ID)
		}
		seen := map[string]bool{}
		for _, id := range task.AllowedVehicles {
			if !vehicleIDs[id] {
				return nil, nil, nil, fmt.Errorf("task %q references unknown vehicle %q", task.ID, id)
			}
			if seen[id] {
				return nil, nil, nil, fmt.Errorf("task %q lists vehicle %q more than once", task.ID, id)
			}
			seen[id] = true
		}
		tasks[i] = task
	}
	return grid, req.Vehicles, tasks, nil
}
