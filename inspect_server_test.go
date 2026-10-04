package main

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"
)

func postInspect(t *testing.T, body string) (int, map[string]interface{}) {
	t.Helper()
	req := httptest.NewRequest(http.MethodPost, "/inspect", bytes.NewBufferString(body))
	rec := httptest.NewRecorder()
	newRouter().ServeHTTP(rec, req)
	var out map[string]interface{}
	_ = json.Unmarshal(rec.Body.Bytes(), &out)
	return rec.Code, out
}

func TestHTTPInspectSuccess(t *testing.T) {
	code, out := postInspect(t, `{
		"width": 5, "height": 1, "max_steps": 24, "node_budget": 1000,
		"vehicles": [{"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 4, "y": 0}}],
		"tasks": [{"id": "j", "work": {"x": 2, "y": 0}, "allowed_vehicles": ["a"],
			"earliest_start": 3, "latest_finish": 8, "dwell_steps": 2}]
	}`)
	if code != http.StatusOK || out["status"] != "success" {
		t.Fatalf("code=%d out=%v", code, out)
	}
	vehicles := out["vehicles"].([]interface{})[0].(map[string]interface{})
	if vehicles["service_start"].(float64) != 3 || vehicles["service_finish"].(float64) != 4 {
		t.Fatalf("window fields wrong: %v", vehicles)
	}
	if vehicles["arrival_time"].(float64) != 6 {
		t.Fatalf("arrival: %v", vehicles)
	}
	if len(vehicles["path"].([]interface{})) != 7 {
		t.Fatalf("path length: %v", vehicles)
	}
	assign := out["assignments"].([]interface{})[0].(map[string]interface{})
	if assign["vehicle_id"] != "a" || assign["task_id"] != "j" {
		t.Fatalf("assignment %v", assign)
	}
}

func TestHTTPInspectBadRequest(t *testing.T) {
	code, out := postInspect(t, `{
		"width": 5, "height": 1, "max_steps": 24, "node_budget": 1000,
		"vehicles": [{"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 4, "y": 0}}],
		"tasks": [{"id": "j", "work": {"x": 9, "y": 0}, "allowed_vehicles": ["a"],
			"earliest_start": 3, "latest_finish": 8, "dwell_steps": 2}]
	}`)
	if code != http.StatusBadRequest || out["error"] == nil {
		t.Fatalf("code=%d out=%v", code, out)
	}
}

func TestHTTPInspectNoSolutionOmitsRoutes(t *testing.T) {
	code, out := postInspect(t, `{
		"width": 3, "height": 1, "max_steps": 16, "node_budget": 1000,
		"vehicles": [
			{"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 2, "y": 0}},
			{"id": "b", "start": {"x": 2, "y": 0}, "goal": {"x": 0, "y": 0}}],
		"tasks": [
			{"id": "t1", "work": {"x": 1, "y": 0}, "allowed_vehicles": ["a"],
				"earliest_start": 0, "latest_finish": 2, "dwell_steps": 2},
			{"id": "t2", "work": {"x": 1, "y": 0}, "allowed_vehicles": ["b"],
				"earliest_start": 0, "latest_finish": 2, "dwell_steps": 2}]
	}`)
	if code != http.StatusOK || out["status"] != "no_solution" {
		t.Fatalf("code=%d out=%v", code, out)
	}
	if _, ok := out["vehicles"]; ok {
		t.Fatalf("no_solution must omit routes: %v", out)
	}
}
