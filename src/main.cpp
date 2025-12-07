

#include <SDL2/SDL.h>
#include "simulation_config.hpp"
#include "renderer.hpp"
#include "simulation.hpp"
#include "naiive_neighbor_search.hpp"


int main() {

    // initialize the renderer
    Renderer renderer; 
    if (!r.init(simulation_config.WINDOW_WIDTH, simulation_config.WINDOW_HEIGHT)) {
        return -1; 
    }

    SimulationState state;
    state.boids.reserve(simulation_config.NUM_BOIDS);

    // initialize boids with random positions and velocities
    for (int i = 0; i < simulation_config.NUM_BOIDS; i++) {
        Boid bird; 
        // random position within window bounds
        bird.x = rand() & simulation_config.WINDOW_WIDTH;
        bird.y = rand() & simulation_config.WINDOW_HEIGHT;
        // random velocity between -0.5 and 0.5
        bird.vx = ((rand() % 100) / 100.0f) - 0.5f;
        bird.vy = ((rand() % 100) / 100.0f) - 0.5f;
        // add bird 
        state.boids.push_back(bird);
    }

    // create neighbor search algorithm
    NaiiveNeighborSearch neighbor_search;
    Simulation Simulation(&neighbor_search);

    // main loop
    bool running = true;
    SDL_Event event;
    Uint32 lasat = SDL_GetTicks();
    while (running) {
        while (SDL_PollEvent(&e)) {
            // handle quit event
            if (e.type == SDL_QUIT) {
                running = false;
            }
            //handle update
            Uint32 now = SDL_GetTicks();
            float dt = (now - last) / 1000.0f; // delta time in seconds
            last = now;
            // update simulation
            Simulation.update(state, dt);
            // render simulation
            renderer.render(state.boids);
        }
    
    // cleanup
    renderer.cleanup();
    return 0;
}