cmake_minimum_required(VERSION 3.16.3)
project(mainTest)

set(TARGET main_test)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)           # Specify C++17 standard
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Make C++17 standard required

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)


set(SOURCES main.cpp Calibration.cpp VisualOdometry.cpp)

include_directories(
    ${OpenCV_INCLUDE_DIRS} # Include OpenCV headers
    ${CERES_INCLUDE_DIRS} # Include Ceres headers
)

add_executable(${TARGET} ${SOURCES})
target_link_libraries(${TARGET} ${OpenCV_LIBS} ${CERES_LIBRARIES})
