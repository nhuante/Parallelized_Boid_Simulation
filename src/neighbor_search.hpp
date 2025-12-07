/*
handles the general interface for neighbor search algorithms
- both the Naive and Grid-based implementations will inherit from this
*/


#pragma once 
#include <vector>
#include "boid.hpp"


class NeighborSearch {
    public:
        virtual ~NeighborSearch() = default; 

        // each derived class will need to implement a search for the boids nearby a given boid
        virtual std::vector<int> get_neighbors(const std::vector<Boid>& boids, 
                                                int boid_index) = 0;

        // called once per simulation step to update grids 
        virtual void build(const std::vector<Boid>& boids) = 0;
};