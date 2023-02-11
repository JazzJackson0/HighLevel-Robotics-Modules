cmake_minimum_required(VERSION 3.16.3)

project(PathPlanning)
add_executable(aStar AStarTest.cpp AStar.cpp)
add_executable(bug BUGTest.cpp BUG.cpp)
add_executable(rrt RRTTest.cpp RRT.cpp)
