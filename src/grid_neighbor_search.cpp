#include "grid_neighbor_search.hpp"
#include "simulation_stats.hpp"
#include <cmath>

void GridNeighborSearch::build(const std::vector<Boid>& boids) {
    // this function will calculate which boids are in which grid cells
    // returns a mapping from cell coordinates to list of boid indices in that cell
    grid.clear();

    for (int i = 0; i < boids.size(); i++) {
        int grid_cell_xpos = static_cast<int>(boids[i].x / simulation_config.GRID_CELL_SIZE);
        int grid_cell_ypos = static_cast<int>(boids[i].y / simulation_config.GRID_CELL_SIZE);
        long long cell_hash = hash_cell(grid_cell_xpos, grid_cell_ypos);
        grid[cell_hash].push_back(i);
    }
}



std::tuple<std::vector<int>, long long> GridNeighborSearch::get_neighbors(const std::vector<Boid>& boids, int index) {
    const Boid& boid = boids[index];

    float perception_radius_sq = simulation_config.PERCEPTION_RADIUS * simulation_config.PERCEPTION_RADIUS;
    int target_grid_cell_xpos = static_cast<int>(boids[index].x / simulation_config.GRID_CELL_SIZE);
    int target_grid_cell_ypos = static_cast<int>(boids[index].y / simulation_config.GRID_CELL_SIZE);

    std::vector<int> neighbors;

    long long checked_candidates = 0; // reset count

    // check only the current cell and the 8 neighboring cells for boids within range
    for (int other_grid_cell_Xoffset = -1; other_grid_cell_Xoffset <= 1; other_grid_cell_Xoffset++) {
        for (int other_grid_cell_Yoffset = -1; other_grid_cell_Yoffset <= 1; other_grid_cell_Yoffset++) {
            // get the hash for the neighboring cell (which contains boid indices that are within that cell)
            long long cell_hash = hash_cell(target_grid_cell_xpos + other_grid_cell_Xoffset,
                                               target_grid_cell_ypos + other_grid_cell_Yoffset);
        
            if (grid.count(cell_hash) == 0) {
                continue; // no boids in this cell, we can skip it
            }    
            
            // the boids in the cell we are currently checking
            const auto& cell_boids = grid[cell_hash];
            
            // only iterate through the birds in the cell to check distance
            for (int boid_index_in_cell : cell_boids) {
                if (boid_index_in_cell == index) continue; // skip self
                checked_candidates++;

                const Boid& other_boid = boids[boid_index_in_cell];

                // calculate squared distance
                float dx = other_boid.x - boid.x;
                float dy = other_boid.y - boid.y;
                float distance_sq = dx*dx + dy*dy;

                if (distance_sq <= perception_radius_sq) {
                    neighbors.push_back(boid_index_in_cell);
                }
            }
        }
    }
    return {neighbors, checked_candidates};

}

