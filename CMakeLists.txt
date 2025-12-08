cmake_minimum_required(VERSION 3.10)
project(ParallelizedBoids CXX)

set(CMAKE_CXX_STANDARD 17)

# Fix WinMain issue for SDL
add_definitions(-DSDL_MAIN_HANDLED)
set(CMAKE_EXE_LINKER_FLAGS "-mconsole")

# SDL2
set(SDL2_INCLUDE_DIR "C:/SDL2/SDL2-2.26.5/i686-w64-mingw32/include/SDL2")
set(SDL2_LIBRARY "C:/SDL2/SDL2-2.26.5/i686-w64-mingw32/lib/libSDL2.dll.a")
set(SDL2MAIN_LIBRARY "C:/SDL2/SDL2-2.26.5/i686-w64-mingw32/lib/libSDL2main.a")

include_directories(${SDL2_INCLUDE_DIR})


# Tell CMake where the source files live
set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")

add_executable(BoidsSim
    ${SRC_DIR}/renderer.cpp
    ${SRC_DIR}/simulation_config.cpp
    ${SRC_DIR}/simulation.cpp
    ${SRC_DIR}/main.cpp
)


# Link SDL2 library
target_link_libraries(BoidsSim  
                        mingw32
                        ${SDL2MAIN_LIBRARY} 
                        ${SDL2_LIBRARY} 
)
