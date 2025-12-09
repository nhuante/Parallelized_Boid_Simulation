/*
Grid Neighbor Search 
- Performance: ???? (I think O(N log N) ?)
- Checks and compares distance from one boid to every other boid in it's own grid cell 
and the neighboring grid cells in the simulation 
*/


#pragma once
#include "neighbor_search.hpp"
#include "simulation_config.hpp"
#include <cmath>



class GridNeighborSearch : public NeighborSearch {
    public:
        // handles building the grid data before neighbors can be queried
        void build(const std::vector<Boid>& boids) override {}

        std::vector<int> get_neighbors(const std::vector<Boid>& boids, 
                                        int boid_index) override {
                                            // FIXME: TO BE IMPLEMENTED LATER
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
};