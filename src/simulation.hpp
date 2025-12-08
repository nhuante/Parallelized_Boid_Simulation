


#pragma once
#include "simulation_state.hpp"
#include "neighbor_search.hpp"

class Simulation {
    private:
        SimulationState state;
        NeighborSearch* neighbor_search = nullptr;

    public:
        Simulation(NeighborSearch* ns) : neighbor_search(ns) {}

        void update(SimulationState& state, float dt);

};
