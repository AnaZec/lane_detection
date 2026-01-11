cmake_minimum_required(VERSION 3.10)
project(lane_finding_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_finding
    src/main.cpp
    src/calibration.cpp
    src/threshold.cpp
    src/perspective.cpp
    src/lane_detect.cpp
    src/pipeline.cpp
)

target_link_libraries(lane_finding ${OpenCV_LIBS})

