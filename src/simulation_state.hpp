/* 
represents the 
*/


#pragma once
#include <vector>
#include "boid.hpp"
#include "simulation_config.hpp"



struct SimulationState {
    SimulationConfig sim_config; 

    std::vector<Boid> boids;
};