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
    float COHESION_WEIGHT = 0.10f;
    float SEPARATION_WEIGHT = 1.50f;


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
};

// Declare a single global instance (so other files can see it exists when they import this header file)
extern SimulationConfig simulation_config;