cmake_minimum_required(VERSION 3.10)
project(BoidsSim)

set(CMAKE_CXX_STANDARD 17)

# Enable OpenMP if available (optional)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

# Find SDL2
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp")

add_executable(BoidsSim ${SOURCES})
target_link_libraries(BoidsSim ${SDL2_LIBRARIES})
