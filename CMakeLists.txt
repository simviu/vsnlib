project(vsnLib)
cmake_minimum_required(VERSION 3.4)

SET(CMAKE_CXX_STANDARD 14)
add_definitions(-fPIC)
#find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
#set( OpenCV_STATIC ON ) 

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (jsoncpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Gflags REQUIRED)

message(STATUS "Eigen3 inc dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "JsonCpp libdir: ${JsonCpp_LIB_DIR}")
message(STATUS "JsonCpp libs: ${jsoncpp_lib}")

include_directories( 
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_DIRS} 
    ${JsonCpp_INCLUDE_DIR}
    #${CMAKE_PREFIX_PATH}/include
    /usr/include/jsoncpp
    /opt/dep/local/include
    inc
    )
#--- Hack for Mac brew
link_directories(/opt/dep/local/lib)

#---- dbg
message(STATUS "OpenCV_dirs:${OpenCV_DIRS}" )
#---- vsnLib 
FILE(GLOB  SRC_VSNLIB src/*.cpp)
add_library(vsnLib ${SRC_VSNLIB})
target_link_libraries(vsnLib   
    ${OpenCV_LIBRARIES} 
    ${Eigen3_LIBRARY}    
    ${JsonCpp_LIBRARIES} 
    -ljsoncpp
    )
set(ENV{vsnLib_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  

#---- vsnTest
FILE(GLOB  SRC_VSNTEST test/*.cpp)
add_executable(vsnTest ${SRC_VSNTEST})
target_link_libraries(vsnTest  ${OpenCV_LIBRARIES} vsnLib)

#---- vsnTool
FILE(GLOB  SRC_VSNTEST apps/*.cpp)
add_executable(vsntool ${SRC_VSNTEST})
target_link_libraries(vsntool  ${OpenCV_LIBRARIES} vsnLib)

