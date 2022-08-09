project(calib)
cmake_minimum_required(VERSION 3.4)

SET(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_DIRS} )

#FILE(GLOB  SRC_ALL *.cpp)
add_executable(calib calib_charuco.cpp)
target_link_libraries(calib   ${OpenCV_LIBRARIES})

