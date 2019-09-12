cmake_minimum_required(VERSION 3.13)
project(OnePtRanSac)

#FIND EIGEN3#
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen found.")
include_directories( ${EIGEN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}/unsupported )

#FIND OPENCV#
find_package(OpenCV 3 QUIET)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message(STATUS "OpenCV 3 found.")
else()
    message(STATUS "OpenCV 3 not found! Please set your OpenCV_DIR!")
endif()

set(CMAKE_CXX_STANDARD 14)

#---------------------------------------------------------------------------------------------------------

#set current DIR
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# get cameras
include_directories(${PROJECT_SOURCE_DIR}/core/include)

# get camera loading system
include_directories(${PROJECT_SOURCE_DIR}/launch_helpers/include)

#set current source files
set( ONEPTRANSAC_SOURCE_FILES src/OnePtRanSac.cpp)

add_library( OnePtRanSac ${ONEPTRANSAC_SOURCE_FILES} )
target_link_libraries( OnePtRanSac ${OpenCV_LIBS} l_cameras launch_helpers)

add_executable(OnePtRanSac_test test/main.cpp)
target_link_libraries( OnePtRanSac_test OnePtRanSac -lpthread)

add_subdirectory(core)
add_subdirectory(launch_helpers)