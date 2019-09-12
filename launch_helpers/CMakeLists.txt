include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set( POLYVIEW_LAUNCH_HELPERS_SOURCE_FILES
    src/launchHelpers.cpp)
  
add_library( launch_helpers ${POLYVIEW_LAUNCH_HELPERS_SOURCE_FILES} )
target_link_libraries( launch_helpers ${OpenCV_LIBS} l_cameras)

