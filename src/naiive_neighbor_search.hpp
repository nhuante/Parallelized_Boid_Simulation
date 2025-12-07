/*
Naiive Neighbor Search 
- Performance: O(N^2)
- Checks and compares distance from one boid to every other boid in the simulation 
*/


#pragma once
#include "neighbor_search.hpp"
#include "simulation_config.hpp"
#include <cmmath>


class NaiiveNeighborSearch : public NeighborSearch {
    public:
        void build(const std::vector<Boid>& boids) override {
            // Naiive neighbor search does not require any precomputation
        }

        std::vector<int> get_neighbors(const std::vector<Boid>& boids, 
                                        int boid_index) override {
            std::vector<int> neighbors;
            const Boid& boid = boids[boid_index];

            // for every other boid, check if it's within perception radius
            for (int i = 0; i < boids.size(); ++i) {
                  if (i == boid_index) continue; // skip self


                  // calculate distance 
                  float dx = boids[i].x - boid.x;
                  float dy = boids[i].y - boid.y;
                  float distance = dx*dx + dy*dy; // squared distance
                  if (distance <= simulation_config.PERCEPTION_RADIUS * simulation_config.PERCEPTION_RADIUS) {
                      neighbors.push_back(i); // if within perception radius, add to neighbors
                  }
            }
            return neighbors;
        }
}