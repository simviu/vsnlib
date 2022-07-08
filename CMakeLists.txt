project(board_det)
cmake_minimum_required(VERSION 3.4)

SET(CMAKE_CXX_STANDARD 14)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_DIRS} inc)

#---- vsnLib
FILE(GLOB  SRC_VSNLIB src/*.cpp)
add_library(vsnLib ${SRC_VSNLIB})
target_link_libraries(vsnLib   ${OpenCV_LIBRARIES} Eigen3::Eigen)
set(ENV{vsnLib_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  
#---- vsnTest
FILE(GLOB  SRC_VSNTEST test/*.cpp)
add_executable(vsnTest ${SRC_VSNTEST})
target_link_libraries(vsnTest vsnLib)

#---- vsnTool
FILE(GLOB  SRC_VSNTEST apps/*.cpp)
add_executable(vsntool ${SRC_VSNTEST})
target_link_libraries(vsntool vsnLib)

