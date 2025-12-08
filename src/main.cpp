#include <SDL.h>
#include "simulation_config.hpp"
#include "renderer.hpp"
#include "simulation.hpp"
#include "naiive_neighbor_search.hpp"

#include <iostream>
using namespace std;

int main() {

    // initialize the renderer
    std::cout << "Initializing Renderer...\n" ;
    Renderer renderer; 
    if (!renderer.init(simulation_config.WINDOW_WIDTH, simulation_config.WINDOW_HEIGHT)) {
        return -1; 
    }
    std::cout << "Done\n" ;

    // initialize simulation state
    std::cout << "Initializing Simulation State for " << simulation_config.NUM_BOIDS << " Boids...\n" ;
    SimulationState state;
    state.boids.reserve(simulation_config.NUM_BOIDS);
    std::cout << "Done\n" ;

    // initialize boids with random positions and velocities
    std::cout << "Randomizing Boid Start Positions...\n" ;
    for (int i = 0; i < simulation_config.NUM_BOIDS; i++) {
        Boid bird; 
        // random position within window bounds
        bird.x = rand() % simulation_config.WINDOW_WIDTH;
        bird.y = rand() % simulation_config.WINDOW_HEIGHT;
        // random velocity between -0.5 and 0.5
        bird.vx = ((rand() % 100) / 100.0f) - 0.5f;
        bird.vy = ((rand() % 100) / 100.0f) - 0.5f;
        // add bird 
        state.boids.push_back(bird);
    }
    std::cout << "Done\n" ;

    // create neighbor search algorithm
    NaiiveNeighborSearch neighbor_search;
    Simulation sim(&neighbor_search);

    // main loop
    bool running = true;
    SDL_Event event;
    Uint32 last = SDL_GetTicks();

    std::cout << "\n\nStarting Simulation...\n" ;
    while (running) {
        
        while (SDL_PollEvent(&event)) {
            // handle quit event
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }
        // calc the timestep
        Uint32 now = SDL_GetTicks();
        float dt = ((now - last) / 1000.0f) * simulation_config.SPEED_UP_RATE; // delta time in seconds
        // std::cout << "\tdt = " << dt << "\n";
        last = now;
        // update simulation
        // std::cout << "Updating...\n";
        sim.update(state, dt);
        // render simulation
        renderer.render(state.boids);
    }
    // cleanup
    renderer.cleanup();
    return 0;
}