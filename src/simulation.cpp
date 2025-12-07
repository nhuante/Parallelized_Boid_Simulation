


#include "simulation.hpp"
#include "simulation_config.hpp"
#include <cmath>


static void limit_speed(Boid& boid) {
    float speed = std::sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
    if (speed > simulation_config.MAX_SPEED) {
        boid.vx = (boid.vx / speed) * simulation_config.MAX_SPEED;
        boid.vy = (boid.vy / speed) * simulation_config.MAX_SPEED;
    }
}


void Simulation::update(SimulationState& state, float dt) {
    std::vector<Boid> boids = state.boids;

    neighbor_search->build(boids);

    std::vector<Boid> new_boids = boids; // copy current boids to update to prevent weird results

    // for each boid, compute the new velocity based on neighbors
    for (int i = 0; i < boids.size(); i++) {
        const Boid& boid = boids[i];
        std::vector<int> neighbors = neighbor_search->get_neighbors(boids, i);
        if (neighbors.empty()) continue; // no neighbors, skip bc no change (it's not being pulled in any direction)

        // initial movement values 
        // (alignment, cohesion, separation)
        float align_vx = 0.0f, align_vy = 0.0f;
        float coh_vx = 0.0f, coh_vy = 0.0f;
        float sep_vx = 0.0f, sep_vy = 0.0f;

        // for each neighbor (that is close enough to affect this boid), calculate how much the boid 
        // should be steered
        for (int neighbor_index : neighbors) {
            // skip self (shouldn't ever run bc we handled this in neighbor search, but just as a sanity check)
            if (neighbor_index == i) continue; 

            const Boid& neighbor = boids[neighbor_index];

            // calc alignment 
            align_vx += neighbor.vx;
            align_vy += neighbor.vy;

            // calc cohesion
            coh_vx += neighbor.x;
            coh_vy += neighbor.y;

            // separation pt1 - calc separation
            float dx = boid.x - neighbor.x;
            float dy = boid.y - neighbor.y;

            // separation pt2 - use inverse square distance for stronger repulsion when closer
            float distance_sq = dx*dx + dy*dy;
            if (distance_sq < 20*20) { // only apply separation if very close
                sep_vx += dx;
                sep_vy += dy;
            }
        }

        int num_neighbors = neighbors.size();

        if (num_neighbors > 0) { // to avoid dividing by zero
            // calc the average alignment considering all neighbors
            align_vx /= num_neighbors;
            align_vy /= num_neighbors;

            // calc the average cohestion considering all neighbors
            coh_vx /= num_neighbors;
            coh_vy /= num_neighbors;
        }

        // Apply weights
        new_boids[i]vx = (align_vx - boid.vx) * simulation_config.ALIGNMENT_WEIGHT;
        new_boids[i]vy = (align_vy - boid.vy) * simulation_config.ALIGNMENT_WEIGHT;

        new_boids[i]vx += (coh_vx - boid.x) * simulation_config.COHESION_WEIGHT;
        new_boids[i]vy += (coh_vy - boid.y) * simulation_config.COHESION_WEIGHT;    

        new_boids[i]vx += sep_vx * 0.05f * simulation_config.SEPARATION_WEIGHT;
        new_boids[i]vy += sep_vy * 0.05f * simulation_config.SEPARATION_WEIGHT;

        limit_speed(new_boids[i]);

        // update position based on new velocity
        new_boids[i].x += new_boids[i]vx * dt;
        new_boids[i].y += new_boids[i]vy * dt;

        // wrap around screen edges
        if (new_boids[i].x < 0) {  // if to left of screen, wrap to right
            new_boids[i].x += simulation_config.WINDOW_WIDTH;
        }
        if (new_boids[i].x >= simulation_config.WINDOW_WIDTH) { // if to right of screen, wrap to left
            new_boids[i].x -= simulation_config.WINDOW_WIDTH;
        }

        boids = new_boids;
    }
}