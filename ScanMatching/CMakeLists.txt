cmake_minimum_required(VERSION 3.16.3)

project(ScanMatching)

# Set Variables
set(TARGET_A icpTest)
set(SOURCES_A ../UnitTests/not_using/ICPTest.cpp ICP.cpp)
# set(CMAKE_BUILD_TYPE Debug)

#Set-up Google Test
set(CMAKE_CXX_STANDARD 14)

include(FetchContent) # https://cmake.org/cmake/help/latest/module/FetchContent.html
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

# Create Executables & Link Dependencies
add_executable(${TARGET_A} ${SOURCES_A})
target_link_libraries(${TARGET_A} PUBLIC gtest_main) # Link Libraries to Executable (Use Google Test's main())
target_link_libraries (${TARGET_A} ${Eigen_LIBRARIES})

# Tell CMake the Target is a Unit Test
add_test(NAME ${TARGET_A} COMMAND ${TARGET_A})