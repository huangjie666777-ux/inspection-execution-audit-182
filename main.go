package main

import (
	"log"
	"net/http"
	"os"
)

func main() {
	port := os.Getenv("PORT")
	if port == "" {
		port = "8080"
	}
	addr := ":" + port
	log.Printf("inspection-fleet-routing listening on %s", addr)
	dataDir := os.Getenv("DATA_DIR")
	app, err := newApp(dataDir, envInt("MAX_EXECUTIONS", defaultMaxExecutions), envInt("MAX_INFLIGHT_PLANS", defaultMaxInFlight))
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("execution data directory: %s", dataDir)
	log.Fatal(http.ListenAndServe(addr, newAppRouter(app)))
}
