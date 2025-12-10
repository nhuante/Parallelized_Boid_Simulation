/* 
settings for the simulation 
- contains parameters like number of boids, max speed, etc.
*/


#pragma once
#include <SDL.h>

struct SimulationConfig {

    // NUMBER OF BOIDS 
    int NUM_BOIDS = 1000;
    int NUM_BOIDS_STEP = 100;

    /* ================= BOID MOVEMENT PARAMETERS ================= */
    // SPEED 
    float MAX_SPEED = 150.0f;                       // maximum speed a boid can travel
    float SPEED = 5.0f;                             // INITIAL VALUE, CAN BE CHANGED DURING SIMULATION
    float SPEED_CHANGE_STEP = 1.0f;                 // amount to increase/decrease speed by

    // PERCEPTION RADIUS (INITIAL VALUES, CAN BE CHANGED DURING SIMULATION)
    float PERCEPTION_RADIUS_STEP = 2.0f;            // amount to increase/decrease perception radius by
    float PERCEPTION_RADIUS = 40.0f;                // radius within which boids consider other boids as neighbors

    // STEERING WEIGHTS (INITIAL VALUES, CAN BE CHANGED DURING SIMULATION)
    float ALIGNMENT_WEIGHT = 0.25f;                 // weight for alignment behavior (aka how significantly boids align their velocity with neighbors)
    float ALIGNMENT_WEIGHT_STEP = 0.05f;            // amount to increase/decrease alignment weight by

    float COHESION_WEIGHT = 0.10f;                  // weight for cohesion behavior (aka how significantly boids move towards the average position of neighbors)
    float COHESION_WEIGHT_STEP = 0.05f;             // amount to increase/decrease cohesion weight by

    float SEPARATION_WEIGHT = 1.50f;                // weight for separation behavior (aka how significantly boids avoid crowding neighbors)
    float SEPARATION_WEIGHT_STEP = 0.1f;            // amount to increase/decrease separation weight by

    /* ================= BOID APPEARANCE PARAMETERS ================= */
    int BOID_WIDTH = 4;                             // width of boid rectangle ** NOTE: to use, must modify renderer.cpp ** 
    int BOID_HEIGHT = 4;                            // height of boid rectangle ** NOTE: to use, must modify renderer.cpp ** 

    SDL_Color BOID_COLOR = {255, 255, 255, 255};    // white color
    SDL_Color BACKGROUND_COLOR = {0, 0, 0, 255};    // black backgrounds

    // triangle boid sizes 
    float BOID_TRIANGLE_SIZE = 5.0f;                // size of the triangle representing the boid


    /* ================= SIMULATION PARAMETERS ================= */
    // widow size
    int WINDOW_WIDTH = 800;                         // width of simulation window
    int WINDOW_HEIGHT = 600;                        // height of simulation window    

    // grid parameters for neighbor search
    float GRID_CELL_SIZE = 60.0f;                   // size of each grid cell for spatial partitioning
    float GRID_CELL_SIZE_STEP = 5.0f;               // amount to increase/decrease grid cell size by

    bool SHOW_GRID = false;                         // whether to render the grid overlay
    bool PAUSED = false;                            // whether the simulation is paused
    bool SHOW_STATS = false;                        // whether to show simulation stats on screen

    // false = Naiive, true = Grid
    bool SIMULATION_TYPE_GRID = false;              // whether to use grid-based neighbor search or naiive search

    bool PARALLELISM_ENABLED = false;               // whether to use parallelism for neighbor search and boid updates
    int PARALLELISM_NUM_THREADS = 4;                // number of threads to use when parallelism is enabled


    /* ================= COMPARISON OPERATORS ================= */
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
               SHOW_STATS == other.SHOW_STATS &&
               SHOW_GRID == other.SHOW_GRID && 
               SIMULATION_TYPE_GRID == other.SIMULATION_TYPE_GRID && 
               PARALLELISM_ENABLED == other.PARALLELISM_ENABLED && 
               PARALLELISM_NUM_THREADS == other.PARALLELISM_NUM_THREADS;
    }

    bool operator!=(const SimulationConfig& other) const {
        return !(*this == other);
    }
};

// Declare a single global instance (so other files can see it exists when they import this header file)
extern SimulationConfig simulation_config;