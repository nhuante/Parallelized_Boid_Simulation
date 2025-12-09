#pragma once
#include <cstdint>

struct SimulationStats {
    float frame_time_ms = 0.0f;
    float update_time_ms = 0.0f;
    float neighbor_time_ms = 0.0f;

    int total_neighbor_checks = 0;
    float avg_neighbors = 0.0f;

    float fps = 0.0f;
};

// Declare a single global instance (so other files can see it exists when they import this header file)
extern SimulationStats simulation_stats;