#include <SDL.h>
#include "simulation_config.hpp"
#include "simulation_stats.hpp"
#include "renderer.hpp"
#include "simulation.hpp"
#include "naiive_neighbor_search.hpp"
#include "grid_neighbor_search.hpp"

#include <iostream>
using namespace std;


SimulationConfig last_simulation_config = {};


void print_simulation_controls_and_state() {
    // // clear console (works on Windows)
    system("cls");
    // std::cout << "\033[H\033[J";   // Move cursor home + clear screen (fast, no flicker)

    std::cout << "============================================================= \n";
    std::cout << "                    Boid Simulation Controls                  \n";
    std::cout << "============================================================= \n";
    std::cout << " Play/Pause                                                   \n";
    std::cout << "     [ P ]                                                    \n";
    std::cout << " Show/Hide UI                                                 \n";
    std::cout << "     [ Q ]                                                    \n";
    std::cout << " Grid Toggle                                                  \n";
    std::cout << "     [ W ]                                                    \n";
    std::cout << " Toggle Neighbor Search Type (Naiive/Grid)                    \n";
    std::cout << "     [ E ]                                                    \n";
    std::cout << " Reset Simulation                                             \n";
    std::cout << "     [ SPACE ]                                                \n";
    std::cout<< " Quit Simulation                                               \n";
    std::cout << "     [ ESC ]                                                  \n";
    std::cout << "=============== Modify Configuration Values===================\n";
    std::cout << "                 [Increase / Decrease]                        \n";
    std::cout << " Grid Cell Size                                               \n";
    std::cout << "     [ J / M ]                                                \n";
    std::cout << " Boids Speed                                                  \n";
    std::cout << "     [ + / - ]                                                \n";
    std::cout << " Boids Count                                                  \n";
    std::cout << "     [ G / B ]                                                \n";
    std::cout << " Perception Radius                                            \n";
    std::cout << "     [ A / Z ]                                                \n";
    // std::cout << "Behavior Weights:                                            \n";
    std::cout << " Alignment   Cohesion   Separation                            \n";
    std::cout << " [ S / X ]   [ D / C ]   [ F / V ]                            \n";
    std::cout << "============================================================= \n";

    if (simulation_config.SIMULATION_TYPE_GRID){
        std::cout << ("   NEIGHBOR SEARCH TYPE: [GRID]");
    } else {
        std::cout << ("   NEIGHBOR SEARCH TYPE: [NAIIVE]");
    }

    if (simulation_config.PAUSED){
        std::cout << ("  STATE: [PAUSED]\n");
    } else {
        std::cout << ("  STATE: [RUNNING]\n");
    }
    
    if (simulation_config.SHOW_STATS){
        std::cout << ("   STATS: [VISIBLE]");
    } else {
        std::cout << ("   STATS: [HIDDEN]");
    }
    if (simulation_config.SHOW_GRID){
        std::cout << ("   GRID: [VISIBLE]\n");
    } else {
        std::cout << ("   GRID: [HIDDEN]\n");
    }

    std::cout << "=============================================================             \n";
    std::cout << "                   CURRENT Simulation State                               \n";
    std::cout << "=============================================================             \n";
    std::cout << "Number of Boids......" << simulation_config.NUM_BOIDS << "                \n";
    std::cout << "Boid Speed..........." << simulation_config.SPEED << "x                   \n";
    std::cout << "Perception Radius...." << simulation_config.PERCEPTION_RADIUS << "        \n\n";

    std::cout << "Alignment Weight....." << simulation_config.ALIGNMENT_WEIGHT << "         \n";
    std::cout << "Cohesion Weight......" << simulation_config.COHESION_WEIGHT << "          \n";
    std::cout << "Separation Weight...." << simulation_config.SEPARATION_WEIGHT << "        \n\n";

    std::cout << "Grid Cell Size......." << simulation_config.GRID_CELL_SIZE << "           \n";
    std::cout << "=============================================================             \n";

    std::cout << "                   CURRENT Simulation Stats                               \n";
    std::cout << "=============================================================             \n";
    std::cout << "FPS....................." << simulation_stats.fps << "                    \n\n";

    std::cout << "Last Frame Time........." << simulation_stats.frame_time_ms << " ms       \n";
    std::cout << "Update Time............." << simulation_stats.update_time_ms << " ms   (" << simulation_stats.percent_update_time << "%)      \n";
    std::cout << "Render Time............." << simulation_stats.render_time_ms << " ms   (" << simulation_stats.percent_render_time << "%)      \n\n";

    std::cout << "Neighbor Calc Time......" << simulation_stats.neighbor_time_ms << " ms    \n";  // time taken to compute who is neighbor to whom ??
    std::cout << "Total Neighbor Checks..." << simulation_stats.total_neighbor_checks << "  \n";  // total number of neighbor checks this frame ??
    std::cout << "Avg Checked Neighbors..." << simulation_stats.avg_checked_neighbors << "   \n"; // average number of boids checked to find neighbors
    std::cout << "Avg Neighbors/Boid......" << simulation_stats.avg_neighbors << "          \n"; // average number of neighbors per boid (those within perception radius)
    std::cout << "=============================================================             \n";
}

void maybe_print_state(Uint64& last_print) {
    // check if any values in simulation_config have changed 
    // or if no print in the last 3 seconds
    Uint32 current_time = SDL_GetTicks();
    if ((simulation_config != last_simulation_config) || (current_time - last_print > 3000)) {
        print_simulation_controls_and_state();
        // update last known config
        last_simulation_config = simulation_config;
        last_print = current_time;
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

    // reset any stats 
    simulation_stats.frame_time_ms = 0.0f;
    simulation_stats.update_time_ms = 0.0f;
    simulation_stats.neighbor_time_ms = 0.0f;
    simulation_stats.total_neighbor_checks = 0;
    simulation_stats.avg_neighbors = 0.0f;
    simulation_stats.fps = 0.0f;
    simulation_stats.checked_neighbors_this_frame = 0;
    simulation_stats.avg_checked_neighbors = 0.0f;
    simulation_stats.render_time_ms = 0.0f;
    simulation_stats.percent_update_time = 0.0f;
    simulation_stats.percent_render_time = 0.0f;
    print_simulation_controls_and_state();
}


void handle_input(const SDL_Event& event, SimulationState& state, Uint32& last_time, Simulation& sim, NeighborSearch*& neighbor_search,
                  NaiiveNeighborSearch& naiive_neighbor_search,
                  GridNeighborSearch& grid_neighbor_search) {
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
            // ================= TOGGLE NEIGHBOR SEARCH TYPE =================
            // [ E ] - toggle neighbor search type
            case SDLK_e:
                // update neighbor search type in the simultion config
                simulation_config.SIMULATION_TYPE_GRID = !simulation_config.SIMULATION_TYPE_GRID;
                // update neighbor search algorithm used in simulation
                if (simulation_config.SIMULATION_TYPE_GRID) { // switch to grid-based search
                    neighbor_search = &grid_neighbor_search;
                    simulation_config.SHOW_GRID = true; // enable grid display when using grid search
                    simulation_config.BOID_COLOR = {38, 43, 214, 255}; // blue boids for grid search
                } else { // switch to naiive search
                    neighbor_search = &naiive_neighbor_search;
                    simulation_config.SHOW_GRID = false; // disable grid display when using naiive search
                    simulation_config.BOID_COLOR = {255, 255, 255, 255}; // white boids for naiive search
                }
                sim.change_neighbor_search_type(neighbor_search);
                break;
            // ================= PAUSE/UNPAUSE TOGGLE =================
            // [ P ] - pause/unpause      
            case SDLK_p:                     
                simulation_config.PAUSED = !simulation_config.PAUSED;
                // turn boids gray when paused 
                if (simulation_config.PAUSED) {
                    simulation_config.BOID_COLOR = {128, 128, 128, 255}; // gray color when paused
                } else {
                    // restore boid color based on neighbor search type
                    if (simulation_config.SIMULATION_TYPE_GRID) {
                        simulation_config.BOID_COLOR = {38, 43, 214, 255}; // blue for grid search
                    } else {
                        simulation_config.BOID_COLOR = {255, 255, 255, 255}; // white for naiive search
                    }
                }
                break;
            // ================= UI CONTROLS =================
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
            // ================= RESET SIMULATION =================
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
    NaiiveNeighborSearch naiive_neighbor_search;
    GridNeighborSearch grid_neighbor_search;
    // default to naiive search
    NeighborSearch* neighbor_search = &naiive_neighbor_search;
    Simulation sim(neighbor_search);

    // main loop
    bool running = true;
    SDL_Event event;
    Uint32 last = SDL_GetTicks();
    Uint64 last_print_time = SDL_GetTicks();


    std::cout << "\n\nStarting Simulation...\n" ;
    print_simulation_controls_and_state();
    while (running) {
        // update and maybe print state if changed
        maybe_print_state(last_print_time);

        // handle events
        while (SDL_PollEvent(&event)) { 
            // handle quit event
            if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                running = false;
            }
            // handle other input
            handle_input(event, state, last, sim, neighbor_search, naiive_neighbor_search, grid_neighbor_search);
        }

        if (simulation_config.PAUSED) {
            // SDL_Delay(10); // sleep to reduce CPU usage when paused
            last = SDL_GetTicks(); // reset last time to prevent large dt jump
            continue;
        }

        // ================= SIMULATION UPDATE START =================
        Uint64 update_frame_start_time = SDL_GetPerformanceCounter();
        // calc the timestep
        Uint32 now = SDL_GetTicks();
        float dt = ((now - last) / 1000.0f) * simulation_config.SPEED; // delta time in seconds
        // std::cout << "\tdt = " << dt << "\n";
        last = now;
        // update simulation
        // std::cout << "Updating...\n";

        // ------------- Simultation Update Start (Calcs) -------------
        Uint64 update_start_time = SDL_GetPerformanceCounter();
        sim.update(state, dt);
        Uint64 update_end_time = SDL_GetPerformanceCounter();
        simulation_stats.update_time_ms = (update_end_time - update_start_time) * 1000.0f / SDL_GetPerformanceFrequency();
        // ------------- Simultation Update End (Calcs) -------------

        // ------------- Render Start -------------
        Uint64 render_start_time = SDL_GetPerformanceCounter();
        renderer.render(state.boids, simulation_config.BACKGROUND_COLOR, simulation_config.BOID_COLOR);
        Uint64 render_end_time = SDL_GetPerformanceCounter();
        simulation_stats.render_time_ms = (render_end_time - render_start_time) * 1000.0f / SDL_GetPerformanceFrequency();
        // ------------- Render End -------------

        Uint64 update_frame_end_time = SDL_GetPerformanceCounter();
        simulation_stats.frame_time_ms = (update_frame_end_time - update_frame_start_time) * 1000.0f / SDL_GetPerformanceFrequency();
        
        // ================= SIMULATION UPDATE END =================
        // update percentages 
        simulation_stats.percent_update_time = (simulation_stats.update_time_ms / simulation_stats.frame_time_ms) * 100.0f;
        simulation_stats.percent_render_time = (simulation_stats.render_time_ms / simulation_stats.frame_time_ms) * 100.0f;
        // update neighbor stats
        simulation_stats.avg_checked_neighbors = (simulation_stats.avg_checked_neighbors + simulation_stats.checked_neighbors_this_frame) / 2.0f;

        // ===================== FPS CALCULATION START ================
        simulation_stats.fps = 1000.0f / simulation_stats.frame_time_ms;
        // ===================== FPS CALCULATION END ================
    }
    // cleanup
    renderer.cleanup();
    return 0;
}