project(calib)
cmake_minimum_required(VERSION 3.4)

SET(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

message(STATUS "Eigen3 inc dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "OpenCV_DIR: ${OpenCV_DIR}")

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    )

#FILE(GLOB  SRC_ALL *.cpp)
add_executable(calib 
    calibrate_camera_charuco.cpp)
target_link_libraries(calib   
    ${OpenCV_LIBRARIES}
    ${Eigen3_LIBRARY}    
    )

