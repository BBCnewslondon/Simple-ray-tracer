cmake_minimum_required(VERSION 3.10)
project(PhysicsSims)

set(CMAKE_CXX_STANDARD 11)
# Allow OpenMP for parallel rendering
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    link_libraries(OpenMP::OpenMP_CXX)
endif()

include_directories(.)

# 1. The Black Hole Project (Old)
add_executable(blackhole main.cpp)

# 2. The Solar System Project (New)
# This compiles 'main_nbody.cpp' into a program called 'nbody'
add_executable(nbody main_nbody.cpp)