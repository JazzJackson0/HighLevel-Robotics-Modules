cmake_minimum_required(VERSION 3.16.3)

project(SLAM)

# Set Variables
set(TARGET_A ekfSLAMTest)
set(SOURCES_A EKFSLAMTest.cpp EKFSLAM.cpp)
set(TARGET_B poseGraphOptSLAMTest)
set(SOURCES_B PoseGraphOptSLAMTest.cpp PoseGraphOptSLAM.cpp)

# Create Executables & Link Dependencies
add_executable(${TARGET_A} ${SOURCES_A})
target_link_libraries(${TARGET_A} PUBLIC gtest_main) # Link Libraries to Executable

add_executable(${TARGET_B} ${SOURCES_B})
target_link_libraries(${TARGET_B} PUBLIC gtest_main) # Link Libraries to Executable

# Tell CMake the Target is a Unit Test
add_test(NAME ${TARGET_A} COMMAND ${TARGET_A})
add_test(NAME ${TARGET_B} COMMAND ${TARGET_B})