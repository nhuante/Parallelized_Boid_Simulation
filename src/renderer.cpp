

#include "simulation_config.hpp"
#include "renderer.hpp"
#include <cmath>


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


void Renderer::render(const std::vector<Boid>& boids, SDL_Color background_color, SDL_Color boid_color) {
    // clear screen
    SDL_SetRenderDrawColor(renderer, background_color.r, background_color.g, background_color.b, 255); // black background
    SDL_RenderClear(renderer);

    // draw each boid [as a rectangle] [NOTE: changed to triangle in draw_boid function]
    // SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // white boids
    // for (const Boid& boid : boids) {
    //     SDL_Rect rect;
    //     rect.x = static_cast<int>(boid.x);
    //     rect.y = static_cast<int>(boid.y);
    //     rect.w = simulation_config.BOID_WIDTH; // width of boid
    //     rect.h = simulation_config.BOID_HEIGHT; // height of boid
    //     SDL_RenderFillRect(renderer, &rect);
    // }

    // draw grid if enabled
    if (simulation_config.SHOW_GRID) {
        draw_grid();
    }

    // render boids as triangles
    for (const Boid& boid : boids){
        float angle = atan2(boid.vy, boid.vx) + M_PI / 2.0f; // add 90 degrees to point in direction of velocity
        SDL_Color color = boid_color; // use passed in boid color
        draw_boid(boid.x, boid.y, angle, color);
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



void Renderer::draw_boid(float x, float y, float angle, SDL_Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);

    float size = simulation_config.BOID_TRIANGLE_SIZE;
    float half = size / 2.0f;

    // this is the first triangle tip (centered and pointing upwards)
    float tip_x = 0.0f;
    float tip_y = -size;

    // this is the bottom left corner
    float bottom_left_x = -half;
    float bottom_left_y = size;

    // this it the bottom right corner
    float bottom_right_x = half;
    float bottom_right_y = size;

    // rotate the points around the boid's current angle 
    float cos_a = cos(angle);
    float sin_a = sin(angle);

    // rotate the tip 
    float rotated_tip_x = x + (tip_x * cos_a - tip_y * sin_a);
    float rotated_tip_y = y + (tip_x * sin_a + tip_y * cos_a);

    // rotate the bottom left
    float rotated_bottom_left_x = x + (bottom_left_x * cos_a - bottom_left_y * sin_a);
    float rotated_bottom_left_y = y + (bottom_left_x * sin_a + bottom_left_y * cos_a);

    // rotate the bottom right
    float rotated_bottom_right_x = x + (bottom_right_x * cos_a - bottom_right_y * sin_a);
    float rotated_bottom_right_y = y + (bottom_right_x * sin_a + bottom_right_y * cos_a);   

    // draw the triangle
    for (float t = 0.0f; t < 1.0f; t += 0.02f) {
        float x1 = rotated_tip_x + (rotated_bottom_left_x - rotated_tip_x) * t;
        float y1 = rotated_tip_y + (rotated_bottom_left_y - rotated_tip_y) * t;

        float x2 = rotated_tip_x + (rotated_bottom_right_x - rotated_tip_x) * t;
        float y2 = rotated_tip_y + (rotated_bottom_right_y - rotated_tip_y) * t;

        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    }

}

void Renderer::draw_grid() {
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255); // dark gray grid lines

    float cell_size = simulation_config.GRID_CELL_SIZE;
    int width = simulation_config.WINDOW_WIDTH;
    int height = simulation_config.WINDOW_HEIGHT;

    // draw vertical lines
    for (float x = 0; x <= width; x += cell_size) {
        SDL_RenderDrawLine(renderer, x, 0, x, height);
    }

    // draw horizontal lines
    for (float y = 0; y <= height; y += cell_size) {
        SDL_RenderDrawLine(renderer, 0, y, width, y);
    }
}