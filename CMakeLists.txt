cmake_minimum_required(VERSION 3.23)
project(test)

set(CMAKE_CXX_STANDARD 14)

add_executable(test main.cpp Leg.cpp Leg.h calculate.cpp calculate.h cppTypes.h cTypes.h)
