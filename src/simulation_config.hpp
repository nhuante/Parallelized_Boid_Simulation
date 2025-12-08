/* 
settings for the simulation 
- contains parameters like number of boids, max speed, etc.
*/


#pragma once

struct SimulationConfig {
    // boid behavior parameters
    int NUM_BOIDS = 5000;
    float MAX_SPEED = 2.50f;
    // float SPEED_UP_RATE = 3.0f;
    float SPEED_UP_RATE = 8.0f;
    float PERCEPTION_RADIUS = 40.0f;

    float ALIGNMENT_WEIGHT = 0.25f;
    float COHESION_WEIGHT = 0.10f;
    float SEPARATION_WEIGHT = 1.50f;

    // boid appearance parameters
    int BOID_WIDTH = 4;
    int BOID_HEIGHT = 4;

    // triangle boid sizes 
    float BOID_TRIANGLE_SIZE = 5.0f;
    
    // widow size
    int WINDOW_WIDTH = 800;
    int WINDOW_HEIGHT = 600;

    // grid parameters for neighbor search
    float GRID_CELL_SIZE = 60.0f;
};

// Declare a single global instance (so other files can see it exists when they import this header file)
extern SimulationConfig simulation_config;