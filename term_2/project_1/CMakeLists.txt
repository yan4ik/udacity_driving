project (ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions (-std=c++0x)

set (sources
     src/main.cpp
     src/Fusion_EKF.cpp
     src/tools.cpp)

add_executable (ExtendedKF ${sources})
