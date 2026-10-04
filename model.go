package main

// Cell is a grid coordinate. X is the column, Y is the row, both 0-based.
type Cell struct {
	X int `json:"x"`
	Y int `json:"y"`
}

// VehicleSpec describes one vehicle in a plan request.
type VehicleSpec struct {
	ID    string `json:"id"`
	Start Cell   `json:"start"`
	Goal  Cell   `json:"goal"`
}

// PlanRequest is the JSON body accepted by POST /plan.
type PlanRequest struct {
	Width      int           `json:"width"`
	Height     int           `json:"height"`
	Obstacles  []Cell        `json:"obstacles"`
	Vehicles   []VehicleSpec `json:"vehicles"`
	MaxSteps   int           `json:"max_steps"`
	NodeBudget int           `json:"node_budget"`
}

// VehiclePath is the planned route of one vehicle.
type VehiclePath struct {
	ID          string `json:"id"`
	Path        []Cell `json:"path"`
	ArrivalTime int    `json:"arrival_time"`
}

// PlanResponse is the JSON body returned by POST /plan.
type PlanResponse struct {
	Status    string        `json:"status"`
	TotalCost int           `json:"total_cost,omitempty"`
	Vehicles  []VehiclePath `json:"vehicles,omitempty"`
}

// TaskSpec describes one inspection job in a POST /inspect request.
type TaskSpec struct {
	ID            string   `json:"id"`
	Cell          Cell     `json:"cell"`
	Vehicles      []string `json:"vehicles"`
	EarliestStart int      `json:"earliest_start"`
	LatestFinish  int      `json:"latest_finish"`
	Dwell         int      `json:"dwell"`
}

// InspectRequest is the JSON body accepted by POST /inspect.
type InspectRequest struct {
	Width      int           `json:"width"`
	Height     int           `json:"height"`
	Obstacles  []Cell        `json:"obstacles"`
	Vehicles   []VehicleSpec `json:"vehicles"`
	Tasks      []TaskSpec    `json:"tasks"`
	MaxSteps   int           `json:"max_steps"`
	NodeBudget int           `json:"node_budget"`
}

// InspectAssignment reports the job and route of one vehicle.
type InspectAssignment struct {
	VehicleID   string `json:"vehicle_id"`
	TaskID      string `json:"task_id"`
	JobStart    int    `json:"job_start"`
	JobFinish   int    `json:"job_finish"`
	Path        []Cell `json:"path"`
	ParkingTime int    `json:"parking_time"`
}

// InspectResponse is the JSON body returned by POST /inspect.
type InspectResponse struct {
	Status      string              `json:"status"`
	TotalCost   int                 `json:"total_cost,omitempty"`
	Assignments []InspectAssignment `json:"assignments,omitempty"`
}

const (
	StatusSuccess    = "success"
	StatusNoSolution = "no_solution"
	StatusUndecided  = "undecided"
)

const (
	maxGridDim    = 20
	maxVehicles   = 5
	maxStepsLimit = 64
	maxBudget     = 20000
)

const (
	maxInspectVehicles = 3
	maxWindowTime      = 64
	minDwell           = 1
	maxDwell           = 8
)

// Grid holds the static map shared by the planners.
type Grid struct {
	W, H    int
	Blocked map[Cell]bool
}

func (g *Grid) free(c Cell) bool {
	return c.X >= 0 && c.X < g.W && c.Y >= 0 && c.Y < g.H && !g.Blocked[c]
}

var dirs = []Cell{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
