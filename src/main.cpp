#include <SDL.h>
#include "simulation_config.hpp"
#include "renderer.hpp"
#include "simulation.hpp"
#include "naiive_neighbor_search.hpp"

#include <iostream>
using namespace std;


void handle_input(const SDL_Event& event, bool& paused, bool& show_stats, float& speed_multiplier) {
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
            // [ P ] - pause/unpause      
            case SDLK_p:                     
                paused = !paused;
                std::cout << (paused ? "Simulation Paused\n" : "Simulation Resumed\n");
                break;
             // [ D ] - show/hide stats 
            case SDLK_d:                         
                show_stats = !show_stats;
                std::cout << (show_stats ? "Showing Stats\n" : "Hiding Stats\n");
                break;          
            // [ + ] - increase speed    
            case SDLK_PLUS: 
            case SDLK_EQUALS:                     
                speed_multiplier += 0.1f;
                std::cout << "Speed Multiplier: " << speed_multiplier << "x\n";
                break;
            // [ - ] - decrease speed
            case SDLK_MINUS:                    
                speed_multiplier = std::max(0.1f, speed_multiplier - 0.1f);
                std::cout << "Speed Multiplier: " << speed_multiplier << "x\n";
                break;
            default:
                break;
        }
    }
}

int main() {

    // game parameters 
    bool paused = false; 
    bool show_stats = false; 
    float speed_multiplier = 1.0f;

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
        // handle events
        while (SDL_PollEvent(&event)) { 
            // handle quit event
            if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                running = false;
            }
            // handle other input
            handle_input(event, paused, show_stats, speed_multiplier);
        }

        if (paused) {
            // SDL_Delay(10); // sleep to reduce CPU usage when paused
            last = SDL_GetTicks(); // reset last time to prevent large dt jump
            continue;
        }

        // calc the timestep
        Uint32 now = SDL_GetTicks();
        float dt = ((now - last) / 1000.0f) * simulation_config.SPEED_UP_RATE * speed_multiplier; // delta time in seconds
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