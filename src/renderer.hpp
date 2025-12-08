/*
handles rendering each step of the simulation 
*/


#pragma once
#include <vector>
#include <SDL.h>
#include "boid.hpp"

class Renderer {
    private:
        SDL_Window* window = nullptr;
        SDL_Renderer* renderer = nullptr;

    public:
        bool init(int width, int height);
        void render(const std::vector<Boid>& boids);
        void draw_boid(float x, float y, float angle, SDL_Color color);
        void cleanup();

};