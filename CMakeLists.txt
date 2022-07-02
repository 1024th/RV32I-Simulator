cmake_minimum_required(VERSION 3.13)
project(RV32I_Simulator)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast")

add_executable(code src/main.cpp)