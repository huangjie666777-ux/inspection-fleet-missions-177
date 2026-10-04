package main

import (
	"encoding/json"
	"net/http"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
)

// newRouter wires the HTTP API. Each request gets an independent planner
// run: constraints and node budget live entirely inside solveCBS.
func newRouter() http.Handler {
	r := chi.NewRouter()
	r.Use(middleware.Recoverer)
	r.Post("/plan", planHandler)
	r.Post("/inspect", inspectHandler)
	r.Get("/healthz", func(w http.ResponseWriter, _ *http.Request) {
		w.WriteHeader(http.StatusOK)
		_, _ = w.Write([]byte("ok"))
	})
	return r
}

func writeJSON(w http.ResponseWriter, code int, v interface{}) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(code)
	_ = json.NewEncoder(w).Encode(v)
}

func planHandler(w http.ResponseWriter, r *http.Request) {
	var req PlanRequest
	dec := json.NewDecoder(r.Body)
	dec.DisallowUnknownFields()
	if err := dec.Decode(&req); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "invalid JSON: " + err.Error()})
		return
	}
	grid, err := validateRequest(&req)
	if err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": err.Error()})
		return
	}

	// r.Context() carries client cancellation: an aborted request stops
	// its CBS search without affecting concurrent requests.
	paths, arrivals, cost, status := solveCBS(r.Context(), grid, req.Vehicles, req.MaxSteps, req.NodeBudget)

	resp := PlanResponse{Status: status}
	if status == StatusSuccess {
		resp.TotalCost = cost
		resp.Vehicles = make([]VehiclePath, len(req.Vehicles))
		for i, v := range req.Vehicles {
			resp.Vehicles[i] = VehiclePath{ID: v.ID, Path: paths[i], ArrivalTime: arrivals[i]}
		}
	}
	writeJSON(w, http.StatusOK, resp)
}

func inspectHandler(w http.ResponseWriter, r *http.Request) {
	var req InspectRequest
	dec := json.NewDecoder(r.Body)
	dec.DisallowUnknownFields()
	if err := dec.Decode(&req); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "invalid JSON: " + err.Error()})
		return
	}
	grid, vehicles, tasks, err := validateInspectRequest(&req)
	if err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": err.Error()})
		return
	}

	plans, assign, cost, status := solveInspect(r.Context(), grid, vehicles, tasks, req.MaxSteps, req.NodeBudget)
	resp := InspectResponse{Status: status}
	if status == StatusSuccess {
		resp.TotalCost = cost
		resp.Assignments = make([]Assignment, len(tasks))
		for ti, vi := range assign {
			resp.Assignments[ti] = Assignment{VehicleID: vehicles[vi].ID, TaskID: tasks[ti].ID}
		}
		resp.Vehicles = make([]InspectionVehiclePath, len(vehicles))
		for vi := range vehicles {
			ti := indexOfAssign(assign, vi)
			p := plans[vi]
			resp.Vehicles[vi] = InspectionVehiclePath{
				ID:            vehicles[vi].ID,
				TaskID:        tasks[ti].ID,
				ServiceStart:  p.servStart,
				ServiceFinish: p.servFin,
				ArrivalTime:   p.arrival,
				Path:          p.path,
			}
		}
	}
	writeJSON(w, http.StatusOK, resp)
}
