#include <SDL.h>
#include "simulation_config.hpp"
#include "renderer.hpp"
#include "simulation.hpp"
#include "naiive_neighbor_search.hpp"

#include <iostream>
using namespace std;


SimulationConfig last_simulation_config = {};


void print_simulation_controls_and_state() {
    // // clear console (works on Windows)
    system("cls");
    // std::cout << "\033[H\033[J";   // Move cursor home + clear screen (fast, no flicker)

    std::cout << "=============================================================\n";
    std::cout << "                    Boid Simulation Controls                 \n";
    std::cout << "=============================================================\n";
    std::cout << "[ P ] - Pause/Unpause Simulation\n";
    std::cout << "[ Q ] - Show/Hide Simulation Stats\n";
    std::cout << "[ W ] - Toggle Grid Display\n";
    std::cout << "[ J ] - Increase Grid Cell Size\n";
    std::cout << "[ M ] - Decrease Grid Cell Size\n";
    std::cout << "[ + ] - Increase Boid Speed\n";
    std::cout << "[ - ] - Decrease Boid Speed\n";
    std::cout << "[ A ] - Increase Perception Radius\n";
    std::cout << "[ R ] - Decrease Perception Radius\n";
    std::cout << "[ S ] - Increase Alignment Weight (Higher means boids align more)\n";
    std::cout << "[ X ] - Decrease Alignment Weight\n";
    std::cout << "[ D ] - Increase Cohesion Weight (Higher means boids pull towards each other more)\n";
    std::cout << "[ C ] - Decrease Cohesion Weight\n";
    std::cout << "[ F ] - Increase Separation Weight (Higher means boids avoid each other more)\n";
    std::cout << "[ V ] - Decrease Separation Weight\n";
    std::cout << "[ G ] - Increase Number of Boids\n";
    std::cout << "[ B ] - Decrease Number of Boids\n";
    std::cout << "[ ESC ] - Quit Simulation\n";
    std::cout << "[ SPACE ] - Reset Simulation\n";
    std::cout << "=============================================================\n";

    if (simulation_config.PAUSED){
        std::cout << (" [SIMULATION PAUSED]");
    } else {
        std::cout << (" [SIMULATION RUNNING]");
    }
    
    if (simulation_config.SHOW_STATS){
        std::cout << (" [STATS VISIBLE]");
    } else {
        std::cout << (" [STATS HIDDEN]");
    }
    if (simulation_config.SHOW_GRID){
        std::cout << (" [GRID VISIBLE]\n");
    } else {
        std::cout << (" [GRID HIDDEN]\n");
    }

    std::cout << "=============================================================\n";
    std::cout << "                     CURRENT Simulation State                \n";
    std::cout << "=============================================================\n";
    std::cout << "Number of Boids: " << simulation_config.NUM_BOIDS << "\n";
    std::cout << "Boid Speed: " << simulation_config.SPEED << "x\n";
    std::cout << "Perception Radius: " << simulation_config.PERCEPTION_RADIUS << "\n\n";

    std::cout << "Alignment Weight: " << simulation_config.ALIGNMENT_WEIGHT << "\n";
    std::cout << "Cohesion Weight: " << simulation_config.COHESION_WEIGHT << "\n";
    std::cout << "Separation Weight: " << simulation_config.SEPARATION_WEIGHT << "\n";

    std::cout << "\nGrid Cell Size: " << simulation_config.GRID_CELL_SIZE << "\n";
    std::cout << "=============================================================\n";
}

void maybe_print_state() {
    // check if any values in simulation_config have changed 
    if (simulation_config != last_simulation_config) {
        print_simulation_controls_and_state();
        // update last known config
        last_simulation_config = simulation_config;
    }
}

void reset_simulation(SimulationState& state) {
    // clear out all boids and reconstruct the array
    state.boids.clear();
    state.boids.reserve(simulation_config.NUM_BOIDS);

    // re-initialize boids with random positions and velocities
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
        // make sure simulation is not paused 
        simulation_config.PAUSED = false;
    }
}


void handle_input(const SDL_Event& event, SimulationState& state, Uint32& last_time) {
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
            // ================= GENERAL CONTROLS =================
            // [ P ] - pause/unpause      
            case SDLK_p:                     
                simulation_config.PAUSED = !simulation_config.PAUSED;
                break;
            // [ Q ] - show/hide stats 
            case SDLK_q:                         
                simulation_config.SHOW_STATS = !simulation_config.SHOW_STATS;
                break;          
            // [ W ] - toggle grid display
            case SDLK_w:
                simulation_config.SHOW_GRID = !simulation_config.SHOW_GRID;
                break;
            // [ J ] - increase grid cell size
            case SDLK_j:                     
                simulation_config.GRID_CELL_SIZE = std::min(simulation_config.GRID_CELL_SIZE + simulation_config.GRID_CELL_SIZE_STEP, simulation_config.WINDOW_HEIGHT / 2.0f);
                break;
            // [ M ] - decrease grid cell size (min 5)
            case SDLK_m:                     
                simulation_config.GRID_CELL_SIZE = std::max(5.0f, simulation_config.GRID_CELL_SIZE - simulation_config.GRID_CELL_SIZE_STEP);
                break;
            // [ SPACE ] - reset simulation 
            case SDLK_SPACE:
                reset_simulation(state);
                last_time = SDL_GetTicks(); // reset last time to prevent large dt jump
                break;
            // ================= SPEED CONTROLS =================
            // [ + ] - increase speed    
            case SDLK_PLUS: 
            case SDLK_EQUALS:                     
                simulation_config.SPEED += simulation_config.SPEED_CHANGE_STEP;
                break;
            // [ - ] - decrease speed (min 0.1x)
            case SDLK_MINUS:                    
                simulation_config.SPEED = std::max(0.1f, simulation_config.SPEED - simulation_config.SPEED_CHANGE_STEP);
                break;
            // ================= BEHAVIOR CONTROLS =================
            // [ A ] - increase perception radius
            case SDLK_a:                     
                simulation_config.PERCEPTION_RADIUS += simulation_config.PERCEPTION_RADIUS_STEP;
                break;
            // [ Z ] - decrease perception radius (min 1.0f)
            case SDLK_z:                     
                simulation_config.PERCEPTION_RADIUS = std::max(1.0f, simulation_config.PERCEPTION_RADIUS - simulation_config.PERCEPTION_RADIUS_STEP);
                break;
            // ================= ALIGNMENT WEIGHT CONTROLS =================
            // [ S ] - increase alignment weight
            case SDLK_s:                     
                simulation_config.ALIGNMENT_WEIGHT += simulation_config.ALIGNMENT_WEIGHT_STEP;
                break;
            // [ X ] - decrease alignment weight (min 0.0f)
            case SDLK_x:                     
                simulation_config.ALIGNMENT_WEIGHT = std::max(0.0f, simulation_config.ALIGNMENT_WEIGHT - simulation_config.ALIGNMENT_WEIGHT_STEP);
                break;
            // ================= COHESION WEIGHT CONTROLS =================
            // [ D ] - increase cohesion weight
            case SDLK_d:                     
                simulation_config.COHESION_WEIGHT += simulation_config.COHESION_WEIGHT_STEP;
                break;
            // [ C ] - decrease cohesion weight (min 0.0f)
            case SDLK_c:                     
                simulation_config.COHESION_WEIGHT = std::max(0.0f, simulation_config.COHESION_WEIGHT - simulation_config.COHESION_WEIGHT_STEP);
                break;
            // ================= SEPARATION WEIGHT CONTROLS =================
            // [ F ] - increase separation weight
            case SDLK_f:                     
                simulation_config.SEPARATION_WEIGHT += simulation_config.SEPARATION_WEIGHT_STEP;
                break;
            // [ V ] - decrease separation weight (min 0.0f)
            case SDLK_v:
                simulation_config.SEPARATION_WEIGHT = std::max(0.0f, simulation_config.SEPARATION_WEIGHT - simulation_config.SEPARATION_WEIGHT_STEP);
                break;
            // ================= NUMBER OF BOIDS CONTROLS =================
            // [ G ] - increase number of boids
            case SDLK_g:                     
                simulation_config.NUM_BOIDS += simulation_config.NUM_BOIDS_STEP; 
                for (int i = 0; i < simulation_config.NUM_BOIDS_STEP; i++) {
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
                break;
            // [ B ] - decrease number of boids
            case SDLK_b:
                simulation_config.NUM_BOIDS = std::max(1, simulation_config.NUM_BOIDS - simulation_config.NUM_BOIDS_STEP); 
                if (state.boids.size() > simulation_config.NUM_BOIDS) {
                    state.boids.resize(simulation_config.NUM_BOIDS); 
                }
                break;
            
            default:
                break;
        }
    }
}

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
    print_simulation_controls_and_state();
    while (running) {
        // update and maybe print state if changed
        maybe_print_state();

        // handle events
        while (SDL_PollEvent(&event)) { 
            // handle quit event
            if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                running = false;
            }
            // handle other input
            handle_input(event, state, last);
        }

        if (simulation_config.PAUSED) {
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