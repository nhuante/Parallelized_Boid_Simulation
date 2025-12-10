


#pragma once
#include "simulation_state.hpp"
#include "neighbor_search.hpp"
#include <list>
using namespace std;

enum class NeighborSearchType {
    NAIIVE,
    GRID
};

class Simulation {
    private:
        SimulationState state;
        NeighborSearchType neighbor_search_type = NeighborSearchType::NAIIVE; // will default to Naiive search first
        NeighborSearch* neighbor_search = nullptr;


    public:
        Simulation(NeighborSearch* ns) : neighbor_search(ns) {}
        void change_neighbor_search_type(NeighborSearch* ns) {
            neighbor_search = ns;
        }
        std::tuple<long long, long long, float> update_void(int index, const std::vector<Boid>& boids, std::vector<Boid>& new_boids, float dt);
        void update(SimulationState& state, float dt);

};
