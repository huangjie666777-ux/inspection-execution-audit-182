package main

import "time"

type Observation struct {
	VehicleID string `json:"vehicle_id"`
	Time      int    `json:"time"`
	Cell      Cell   `json:"cell"`
}

type ObservationBatch struct {
	Observations []Observation `json:"observations"`
}

type PlannedVehicle struct {
	VehicleID   string `json:"vehicle_id"`
	TaskID      string `json:"task_id"`
	JobStart    int    `json:"job_start"`
	JobFinish   int    `json:"job_finish"`
	ParkingTime int    `json:"parking_time"`
	Path        []Cell `json:"path"`
}

type ExecutionPlan struct {
	Status    string           `json:"status"`
	TotalCost int              `json:"total_cost"`
	MaxSteps  int              `json:"max_steps"`
	Width     int              `json:"width"`
	Height    int              `json:"height"`
	Obstacles []Cell           `json:"obstacles"`
	Tasks     []TaskSpec       `json:"tasks"`
	Vehicles  []PlannedVehicle `json:"vehicles"`
}

type MissingInterval struct {
	VehicleID string `json:"vehicle_id"`
	Start     int    `json:"start"`
	End       int    `json:"end"`
}

type Evidence struct {
	Type       string   `json:"type"`
	VehicleID  string   `json:"vehicle_id,omitempty"`
	VehicleIDs []string `json:"vehicle_ids,omitempty"`
	Time       int      `json:"time,omitempty"`
	FromTime   int      `json:"from_time,omitempty"`
	ToTime     int      `json:"to_time,omitempty"`
	Cell       *Cell    `json:"cell,omitempty"`
	From       *Cell    `json:"from,omitempty"`
	To         *Cell    `json:"to,omitempty"`
	Expected   *Cell    `json:"expected,omitempty"`
	Actual     *Cell    `json:"actual,omitempty"`
}

type TaskResult struct {
	TaskID         string `json:"task_id"`
	VehicleID      string `json:"vehicle_id"`
	Status         string `json:"status"`
	EarliestStart  int    `json:"earliest_start,omitempty"`
	EarliestFinish int    `json:"earliest_finish,omitempty"`
}

type Report struct {
	Status           string            `json:"status"`
	ObservationCount int               `json:"observation_count"`
	DeviationCount   int               `json:"deviation_count"`
	Deviations       []Evidence        `json:"deviations"`
	MissingIntervals []MissingInterval `json:"missing_intervals"`
	TaskResults      []TaskResult      `json:"task_results"`
	Evidence         []Evidence        `json:"evidence"`
}

type ObservedPoint struct {
	Time int  `json:"time"`
	Cell Cell `json:"cell"`
}

type ExecutionRecord struct {
	ID           string                     `json:"id"`
	CreatedAt    time.Time                  `json:"created_at"`
	Sealed       bool                       `json:"sealed"`
	SealedAt     *time.Time                 `json:"sealed_at,omitempty"`
	Request      InspectRequest             `json:"request"`
	Plan         ExecutionPlan              `json:"plan"`
	Observations map[string][]ObservedPoint `json:"observations"`
	Report       *Report                    `json:"report,omitempty"`
}

type ExecutionSnapshot struct {
	ID           string                     `json:"id"`
	CreatedAt    time.Time                  `json:"created_at"`
	Sealed       bool                       `json:"sealed"`
	SealedAt     *time.Time                 `json:"sealed_at,omitempty"`
	Plan         ExecutionPlan              `json:"plan"`
	Observations map[string][]ObservedPoint `json:"observations"`
	Report       Report                     `json:"report"`
}

type ObservationResponse struct {
	ExecutionID string `json:"execution_id"`
	Accepted    int    `json:"accepted"`
	Duplicate   int    `json:"duplicate"`
	Report      Report `json:"report"`
}

type CreateExecutionResponse struct {
	ExecutionID string         `json:"execution_id,omitempty"`
	Status      string         `json:"status"`
	Plan        *ExecutionPlan `json:"plan,omitempty"`
}

const (
	TaskCompleted  = "completed"
	TaskIncomplete = "incomplete"
	TaskUnknown    = "unknown"
	ReportPassed   = "passed"
	ReportFailed   = "failed"
	ReportUnknown  = "unknown"
)
