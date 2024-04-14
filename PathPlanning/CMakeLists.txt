cmake_minimum_required(VERSION 3.16.3)

project(PathPlanning)

# Set Variables
set(TARGET_A aStarTest)
set(SOURCES_A ../UnitTests/AStarTest.cpp AStar.cpp)
set(TARGET_B bugTest)
set(SOURCES_B ../UnitTests/BUGTest.cpp BUG.cpp)
set(TARGET_C rrtTest)
set(SOURCES_C ../UnitTests/RRTTest.cpp RRT.cpp)

#Set-up Google Test
set(CMAKE_CXX_STANDARD 14)

include(FetchContent)  # https://cmake.org/cmake/help/latest/module/FetchContent.html
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

include_directories(${CMAKE_SOURCE_DIR}/../DataStructures/)

# Create Executables & Link Dependencies
add_executable(${TARGET_A} ${SOURCES_A})
target_link_libraries(${TARGET_A} PUBLIC gtest_main) # Link GoogleTest's main() to Executable

add_executable(${TARGET_B} ${SOURCES_B})
target_link_libraries(${TARGET_B} PUBLIC gtest_main) # Link GoogleTest's main() to Executable

add_executable(${TARGET_C} ${SOURCES_C})
target_link_libraries(${TARGET_C} PUBLIC gtest_main) # Link GoogleTest's main() to Executable


# Tell CMake the Target is a Unit Test
add_test(NAME ${TARGET_A} COMMAND ${TARGET_A})
add_test(NAME ${TARGET_B} COMMAND ${TARGET_B})
add_test(NAME ${TARGET_C} COMMAND ${TARGET_C})