

#include "simulation_config.hpp"
#include "renderer.hpp"


bool Renderer::init(int width, int height) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0){
        return false;
    }

    window = SDL_CreateWindow("Boid Simulation", 
                              SDL_WINDOWPOS_CENTERED, 
                              SDL_WINDOWPOS_CENTERED, 
                              width, height, 
                              SDL_WINDOW_SHOWN);

    if (!window) {
        return false; 
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        return false; 
    }

    return true;
}


void Renderer::render(const std::vector<Boid>& boids) {
    // clear screen
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // black background
    SDL_RenderClear(renderer);

    // draw each boid
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // white boids
    for (const Boid& boid : boids) {
        SDL_Rect rect;
        rect.x = static_cast<int>(boid.x);
        rect.y = static_cast<int>(boid.y);
        rect.w = config.BOID_WIDTH; // width of boid
        rect.h = config.BOID_HEIGHT; // height of boid
        SDL_RenderFillRect(renderer, &rect);
    }

    // present the rendered frame
    SDL_RenderPresent(renderer);
}


void Renderer::cleanup() {
    if (renderer) {
        SDL_DestroyRenderer(renderer);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
}