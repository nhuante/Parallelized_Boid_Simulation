/* 
settings for the simulation 
- contains parameters like number of boids, max speed, etc.
*/


#pragma once

struct SimulationConfig {

    // NUMBER OF BOIDS 
    int NUM_BOIDS = 1000;

    /* ================= BOID MOVEMENT PARAMETERS ================= */
    // SPEED
    float MAX_SPEED = 150.0f;
    float SPEED = 2.0f;
    float SPEED_CHANGE_STEP = 0.5f;

    // PERCEPTION RADIUS 
    float PERCEPTION_RADIUS_STEP = 2.0f;
    float PERCEPTION_RADIUS = 40.0f;

    // STEERING WEIGHTS
    float ALIGNMENT_WEIGHT = 0.25f;
    float ALIGNMENT_WEIGHT_STEP = 0.05f;

    float COHESION_WEIGHT = 0.10f;
    float COHESION_WEIGHT_STEP = 0.05f;

    float SEPARATION_WEIGHT = 1.50f;
    float SEPARATION_WEIGHT_STEP = 0.1f;

    /* ================= BOID APPEARANCE PARAMETERS ================= */
    int BOID_WIDTH = 4;
    int BOID_HEIGHT = 4;

    // triangle boid sizes 
    float BOID_TRIANGLE_SIZE = 5.0f;


    /* ================= SIMULATION PARAMETERS ================= */
    // widow size
    int WINDOW_WIDTH = 800;
    int WINDOW_HEIGHT = 600;

    // grid parameters for neighbor search
    float GRID_CELL_SIZE = 60.0f;

    bool PAUSED = false;
    bool SHOW_STATS = false;

    bool operator==(const SimulationConfig& other) const {
        return NUM_BOIDS == other.NUM_BOIDS &&
               MAX_SPEED == other.MAX_SPEED &&
               SPEED == other.SPEED &&
               SPEED_CHANGE_STEP == other.SPEED_CHANGE_STEP &&
               PERCEPTION_RADIUS_STEP == other.PERCEPTION_RADIUS_STEP &&
               PERCEPTION_RADIUS == other.PERCEPTION_RADIUS &&
               ALIGNMENT_WEIGHT == other.ALIGNMENT_WEIGHT &&
               ALIGNMENT_WEIGHT_STEP == other.ALIGNMENT_WEIGHT_STEP &&
               COHESION_WEIGHT == other.COHESION_WEIGHT &&
               COHESION_WEIGHT_STEP == other.COHESION_WEIGHT_STEP &&
               SEPARATION_WEIGHT == other.SEPARATION_WEIGHT &&
               SEPARATION_WEIGHT_STEP == other.SEPARATION_WEIGHT_STEP &&
               BOID_WIDTH == other.BOID_WIDTH &&
               BOID_HEIGHT == other.BOID_HEIGHT &&
               BOID_TRIANGLE_SIZE == other.BOID_TRIANGLE_SIZE &&
               WINDOW_WIDTH == other.WINDOW_WIDTH &&
               WINDOW_HEIGHT == other.WINDOW_HEIGHT &&
               GRID_CELL_SIZE == other.GRID_CELL_SIZE &&
               PAUSED == other.PAUSED &&
               SHOW_STATS == other.SHOW_STATS;
    }

    bool operator!=(const SimulationConfig& other) const {
        return !(*this == other);
    }
};

// Declare a single global instance (so other files can see it exists when they import this header file)
extern SimulationConfig simulation_config;