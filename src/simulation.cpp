

#include <omp.h>
#include "simulation.hpp"
#include "simulation_config.hpp"
#include "simulation_stats.hpp"
#include <cmath>
#include <SDL.h>


static void limit_speed(Boid& boid) {
    float speed = std::sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
    if (speed > simulation_config.MAX_SPEED) {
        boid.vx = (boid.vx / speed) * simulation_config.MAX_SPEED;
        boid.vy = (boid.vy / speed) * simulation_config.MAX_SPEED;
    }
}

void Simulation::update_void(int i, const std::vector<Boid>& boids, std::vector<Boid>& new_boids, int& total_neighbor_checks, float dt) {
    const Boid& boid = boids[i];
    // ================= GET NEIGHBORS START =================
    Uint64 ns_start_time = SDL_GetPerformanceCounter();
    std::vector<int> neighbors = neighbor_search->get_neighbors(boids, i);
    Uint64 ns_end_time = SDL_GetPerformanceCounter();
    simulation_stats.get_neighbors_calc_time_ms = (ns_end_time - ns_start_time) * 1000.0f / SDL_GetPerformanceFrequency();
    // ================= GET NEIGHBORS END =================

    // TRACK NEIGHBOR STATS 
    total_neighbor_checks += neighbors.size();

    // initial steering shifts 
    float steer_x = 0.0f;
    float steer_y = 0.0f;

    // only compute steering according to other boids IF there are neighbors
    if (!neighbors.empty()) {
        // (alignment, cohesion, separation)
        float align_x = 0.0f, align_y = 0.0f;
        float coh_x = 0.0f, coh_y = 0.0f;
        float sep_x = 0.0f, sep_y = 0.0f;

        // for each neighbor (that is close enough to affect this boid), calculate how much the boid 
        // should be steered
        for (int neighbor_index : neighbors) {
            // skip self (shouldn't ever run bc we handled this in neighbor search, but just as a sanity check)
            if (neighbor_index == i) continue; 

            const Boid& neighbor = boids[neighbor_index];

            // calc alignment 
            align_x += neighbor.vx;
            align_y += neighbor.vy;

            // calc cohesion
            coh_x += neighbor.x;
            coh_y += neighbor.y;

            // separation pt1 - calc separation
            float dx = boid.x - neighbor.x;
            float dy = boid.y - neighbor.y;

            // separation pt2 - use inverse square distance for stronger repulsion when closer
            float distance_sq = dx*dx + dy*dy;
            if (distance_sq < 0.0001f) distance_sq = 0.0001f; // prevent division by zero
            // apply inverse-square separation (stronger repulsion when closer, like magnets)
            sep_x += dx / distance_sq;
            sep_y += dy / distance_sq;

        }

        int num_neighbors = neighbors.size();

        if (num_neighbors > 0) { // to avoid dividing by zero
            // calc the average alignment considering all neighbors
            align_x /= num_neighbors;
            align_y /= num_neighbors;

            // calc the average cohestion considering all neighbors
            coh_x /= num_neighbors;
            coh_y /= num_neighbors;
            // move towards the average position of neighbors
            coh_x -= boid.x;
            coh_y -= boid.y;
        }

        // Apply weights
        steer_x += (align_x - boid.vx) * simulation_config.ALIGNMENT_WEIGHT;
        steer_y += (align_y - boid.vy) * simulation_config.ALIGNMENT_WEIGHT;

        steer_x += (coh_x) * simulation_config.COHESION_WEIGHT;
        steer_y += (coh_y) * simulation_config.COHESION_WEIGHT;

        steer_x += (sep_x) * simulation_config.SEPARATION_WEIGHT;
        steer_y += (sep_y) * simulation_config.SEPARATION_WEIGHT;
    }

    // add steering onto existing velocity
    new_boids[i].vx = boid.vx + steer_x;
    new_boids[i].vy = boid.vy + steer_y;

    limit_speed(new_boids[i]);

    // update position based on new velocity
    new_boids[i].x += new_boids[i].vx * dt;
    new_boids[i].y += new_boids[i].vy * dt;

    // wrap around screen edges
    if (new_boids[i].x < 0) {                               // if to left of screen, wrap to right
        new_boids[i].x += simulation_config.WINDOW_WIDTH;
    }
    if (new_boids[i].x >= simulation_config.WINDOW_WIDTH) { // if to right of screen, wrap to left
        new_boids[i].x -= simulation_config.WINDOW_WIDTH;
    }
    if (new_boids[i].y < 0) {                               // if above screen, wrap to bottom
        new_boids[i].y += simulation_config.WINDOW_HEIGHT;
    }
    if (new_boids[i].y >= simulation_config.WINDOW_HEIGHT) { // if below screen, wrap to top
        new_boids[i].y -= simulation_config.WINDOW_HEIGHT;
    }
}


void Simulation::update(SimulationState& state, float dt) {
    std::vector<Boid> boids = state.boids;
    simulation_stats.checked_neighbors_this_frame = 0;
    
    // ================= CALCULATE NEIGHBORS START =================
    Uint64 start_time = SDL_GetPerformanceCounter();
    neighbor_search->build(boids);
    Uint64 end_time = SDL_GetPerformanceCounter();
    simulation_stats.grid_map_hash_time_ms = (end_time - start_time) * 1000.0f / SDL_GetPerformanceFrequency();
    // ================= CALCULATE NEIGHBORS END =================

    std::vector<Boid> new_boids = boids; // copy current boids to update to prevent weird results
    int total_neighbor_checks = 0;

    if (simulation_config.PARALLELISM_ENABLED) {
        #pragma omp parallel 
        {
            // ================ PARALLEL VERSION START ================
            // record number of threads used
            #pragma omp master 
            simulation_config.PARALLELISM_NUM_THREADS = omp_get_num_threads();

            // for each boid, compute the new velocity based on neighbors (we can split this computation across threads)
            #pragma omp for schedule(dynamic)
            for (int i = 0; i < boids.size(); i++) {
                update_void(i, boids, new_boids, total_neighbor_checks, dt);
            }
            // ================ PARALLEL VERSION END ================
        }
    }
    else {
        // ================ SERIAL VERSION START ================
        simulation_config.PARALLELISM_NUM_THREADS = 1;
        // for each boid, compute the new velocity based on neighbors
        for (int i = 0; i < boids.size(); i++) {
            update_void(i, boids, new_boids, total_neighbor_checks, dt);
         }
        // ================ SERIAL VERSION END ================
    }

    // after all boids updated, update stats
    simulation_stats.total_neighbor_checks = total_neighbor_checks;
    simulation_stats.avg_neighbors = (total_neighbor_checks) / boids.size();
    // update the simulation state with new boid positions and velocities
    state.boids = new_boids;
}