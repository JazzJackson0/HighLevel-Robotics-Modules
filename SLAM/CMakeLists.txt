cmake_minimum_required(VERSION 3.16.3)

project(SLAM)

include_directories(${CMAKE_SOURCE_DIR}/../DataStructures/)
include_directories(${CMAKE_SOURCE_DIR}/../ScanMatching/)

# Set Variables
set(TARGET_A ekfSLAMTest)
set(SOURCES_A EKFSLAMTest.cpp EKFSLAM.cpp)
set(TARGET_B poseGraphOptSLAMTest)
set(SOURCES_B PoseGraphOptSLAMTest.cpp PoseGraphOptSLAM.cpp ${CMAKE_SOURCE_DIR}/../ScanMatching/ICP.cpp)

#Set-up Google Test
set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

# Eigen Set-Up
#find_package(Eigen3 REQUIRED)
#link_directories(${Eigen_INCLUDE_DIRS})

# Set Options for CppAD
#find_package(CppAD REQUIRED)

# Create Executables & Link Dependencies
add_executable(${TARGET_A} ${SOURCES_A})
target_link_libraries(${TARGET_A} PUBLIC gtest_main) # Link GoogleTest's main() to Executable
target_link_libraries (${TARGET_A} ${Eigen_LIBRARIES})

add_executable(${TARGET_B} ${SOURCES_B})
target_link_libraries(${TARGET_B} PUBLIC gtest_main) # Link GoogleTest's main() to Executable
target_link_libraries (${TARGET_B} ${Eigen_LIBRARIES})

# Tell CMake the Target is a Unit Test
add_test(NAME ${TARGET_A} COMMAND ${TARGET_A})
add_test(NAME ${TARGET_B} COMMAND ${TARGET_B})
