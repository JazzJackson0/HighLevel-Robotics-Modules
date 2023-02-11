cmake_minimum_required(VERSION 3.16.3)

project(SLAM)
add_executable(ekfSLAM EKFSLAMTest.cpp EKFSLAM.cpp)
add_executable(poseGraphOptSLAM PoseGraphOptSLAMTest.cpp PoseGraphOptSLAM.cpp)
