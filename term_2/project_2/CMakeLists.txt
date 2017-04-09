project (UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions (-std=c++0x)

set (sources
     src/main.cpp
     src/Fusion_UKF.cpp
     src/tools.cpp)

add_executable (UnscentedKF ${sources})
