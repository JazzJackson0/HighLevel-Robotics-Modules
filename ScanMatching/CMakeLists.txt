cmake_minimum_required(VERSION 3.16.3)

project(ScanMatching)

# Set Variables
set(TARGET_A icpTest)
set(SOURCES_A ICPTest.cpp ICP.cpp)

# Create Executables & Link Dependencies
add_executable(${TARGET_A} ${SOURCES_A})
target_link_libraries(${TARGET_A} PUBLIC gtest_main) # Link Libraries to Executable

# Tell CMake the Target is a Unit Test
add_test(NAME ${TARGET_A} COMMAND ${TARGET_A})