package main

import (
	"log"
	"net/http"
	"os"
	"strconv"
)

func main() {
	port := os.Getenv("PORT")
	if port == "" {
		port = "8080"
	}
	store := newExecStore(
		os.Getenv("EXEC_DATA_DIR"),
		envInt("EXEC_MAX_EXECUTIONS", defaultMaxExecutions),
		envInt("EXEC_MAX_BATCH", defaultMaxBatch),
	)
	if err := store.load(); err != nil {
		log.Fatalf("loading executions: %v", err)
	}
	addr := ":" + port
	log.Printf("inspection-fleet-routing listening on %s", addr)
	log.Fatal(http.ListenAndServe(addr, newRouterWithStore(store)))
}

func envInt(key string, fallback int) int {
	if v := os.Getenv(key); v != "" {
		if n, err := strconv.Atoi(v); err == nil && n > 0 {
			return n
		}
	}
	return fallback
}
