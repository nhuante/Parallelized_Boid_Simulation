/* 
settings for the simulation 
- contains parameters like number of boids, max speed, etc.
*/


#pragma once


struct SimulationConfig {
    // boid behavior parameters
    int NUM_BOIDS = 200;
    float MAX_SPEED = 2.5f;
    float PERCEPTION_RADIUS = 50.0f;
    float ALIGNMENT_WEIGHT = 1.0f;
    float COHESION_WEIGHT = 1.0f;
    float SEPARATION_WEIGHT = 1.5f;

    // boid appearance parameters
    int BOID_WIDTH = 4;
    int BOID_HEIGHT = 4;
    

    // widow size
    int WINDOW_WIDTH = 1280;
    int WINDOW_HEIGHT = 720;

    // grid parameters for neighbor search
    float GRID_CELL_SIZE = 60.0f;
}