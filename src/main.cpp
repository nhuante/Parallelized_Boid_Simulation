#include <SDL.h>
#include "simulation_config.hpp"
#include "renderer.hpp"
#include "simulation.hpp"
#include "naiive_neighbor_search.hpp"

#include <iostream>
using namespace std;

void welcome_message() {
    std::cout << "=========================================\n";
    std::cout << "        Boid Simulation Controls         \n";
    std::cout << "=========================================\n";
    std::cout << "[ P ] - Pause/Unpause Simulation\n";
    std::cout << "[ D ] - Show/Hide Simulation Stats\n";
    std::cout << "[ + ] - Increase Boid Speed\n";
    std::cout << "[ - ] - Decrease Boid Speed\n";
    std::cout << "[ R ] - Increase Perception Radius\n";
    std::cout << "[ F ] - Decrease Perception Radius\n";
    std::cout << "[ ESC ] - Quit Simulation\n";
    std::cout << "=========================================\n\n";

    std::cout << "=========================================\n";
    std::cout << "        Initial Simulation State         \n";
    std::cout << "=========================================\n";
    std::cout << "Number of Boids: " << simulation_config.NUM_BOIDS << "\n";
    std::cout << "Boid Speed: " << simulation_config.SPEED << "x\n";
    std::cout << "Perception Radius: " << simulation_config.PERCEPTION_RADIUS << "\n";
    std::cout << "=========================================\n";
}

void handle_input(const SDL_Event& event, bool& paused, bool& show_stats) {
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
                simulation_config.SPEED += simulation_config.SPEED_CHANGE_STEP;
                std::cout << "Boid Speed: " << simulation_config.SPEED << "x\n";
                break;
            // [ - ] - decrease speed (min 0.1x)
            case SDLK_MINUS:                    
                simulation_config.SPEED = std::max(0.1f, simulation_config.SPEED - simulation_config.SPEED_CHANGE_STEP);
                std::cout << "Boid Speed: " << simulation_config.SPEED << "x\n";
                break;
            // [ R ] - increase perception radius
            case SDLK_r:                     
                simulation_config.PERCEPTION_RADIUS += simulation_config.PERCEPTION_RADIUS_STEP;
                std::cout << "Perception Radius: " << simulation_config.PERCEPTION_RADIUS << "\n";
                break;
            // [ F ] - decrease perception radius (min 1.0f)
            case SDLK_f:                     
                simulation_config.PERCEPTION_RADIUS = std::max(1.0f, simulation_config.PERCEPTION_RADIUS - simulation_config.PERCEPTION_RADIUS_STEP);
                std::cout << "Perception Radius: " << simulation_config.PERCEPTION_RADIUS << "\n";
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

    // welcome message 
    welcome_message(); 

    std::cout << "\n\nStarting Simulation...\n" ;
    while (running) {
        // handle events
        while (SDL_PollEvent(&event)) { 
            // handle quit event
            if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                running = false;
            }
            // handle other input
            handle_input(event, paused, show_stats);
        }

        if (paused) {
            // SDL_Delay(10); // sleep to reduce CPU usage when paused
            last = SDL_GetTicks(); // reset last time to prevent large dt jump
            continue;
        }

        // calc the timestep
        Uint32 now = SDL_GetTicks();
        float dt = ((now - last) / 1000.0f) * simulation_config.SPEED; // delta time in seconds
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