project (Kidnapped_Vehichle)

cmake_minimum_required (VERSION 3.5)

add_definitions (-std=c++0x)

set (sources
     src/main.cpp
     src/particle_filter.cpp)

add_executable (Kidnapped_Vehichle ${sources})
