cmake_minimum_required(VERSION 3.16.3)

project(FeatureExtractor)

# Set Variables
set(TARGET_A featureExtractionTest)
set(SOURCES_A ../UnitTests/FeatureExtractionTest.cpp FeatureExtraction.cpp utils.cpp)

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
target_link_libraries(${TARGET_A} PUBLIC gmock_main) # Link Libraries to Executable
#target_link_libraries(${TARGET_A} PUBLIC gtest_main) # Use if not using gmock

# gmock_main ^ provides a main() function. Debuggers will access the program through this main().



# Tell CMake the Target is a Unit Test
add_test(NAME ${TARGET_A} COMMAND ${TARGET_A})