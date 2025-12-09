/*
Grid Neighbor Search 
- Performance: ???? (I think O(N log N) ?)
- Checks and compares distance from one boid to every other boid in it's own grid cell 
and the neighboring grid cells in the simulation 
*/


#pragma once
#include "neighbor_search.hpp"
#include "simulation_config.hpp"
#include <unordered_map>
#include <cmath>
using namespace std;




class GridNeighborSearch : public NeighborSearch {
    public:
        
        // handles building the grid data before neighbors can be queried
        void build(const std::vector<Boid>& boids) override;
        std::vector<int> get_neighbors(const std::vector<Boid>& boids, int index) override;

    private:
        std::unordered_map<long long, std::vector<int>> grid; 
        // map from cell hash to list of boid indices

        long long hash_cell(int gx, int gy) const {
            return (static_cast<long long>(gx) << 32) | static_cast<unsigned int>(gy);
        }
};