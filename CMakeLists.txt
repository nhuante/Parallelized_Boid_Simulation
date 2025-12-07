cmake_minimum_required(VERSION 3.10)
project(ParallelizedBoids CXX)

set(CMAKE_CXX_STANDARD 17)


# Tell CMake where the source files live
set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")


add_executable(BoidsSim
    ${SRC_DIR}/boid.hpp
    ${SRC_DIR}/renderer.hpp
    ${SRC_DIR}/renderer.cpp
    ${SRC_DIR}/simulation_config.hpp
    ${SRC_DIR}/simulation_state.hpp
    ${SRC_DIR}/simulation.hpp
    ${SRC_DIR}/simulation.cpp
    ${SRC_DIR}/neighbor_search.hpp
    ${SRC_DIR}/naiive_neighbor_search.hpp
    ${SRC_DIR}/main.cpp
)
