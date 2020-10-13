cmake_minimum_required(VERSION 2.6)
enable_testing()
project(Robot_simulator)



set(CMAKE_CXX_STANDARD 14)

#include_directories(.)

add_executable(Robot_simulator
        main.cpp
        matplotlibcpp.h
        robot.h)

find_package(Python2 COMPONENTS Development NumPy matplotlib)
target_include_directories(Robot_simulator PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} )
target_link_libraries(Robot_simulator Python2::Python Python2::NumPy )