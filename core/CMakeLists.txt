include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set( POLYVIEW_SOURCE_FILES
    src/BaseClass.cpp
    src/cameras/Camera.cpp
    src/cameras/Perspective.cpp
    src/cameras/Ocam.cpp
    src/cameras/CameraSystem.cpp
    src/cameras/OmniProjection.cpp
    src/cameras/NoDistortion.cpp
    src/cameras/EquidistantDistortion.cpp
    src/cameras/RadialTangentialDistortion.cpp
    src/tools/Histogram.cpp
    src/tools/options.cpp
    src/tools/cayley.cpp
    src/tools/random_generators.cpp)
add_library( l_cameras ${POLYVIEW_SOURCE_FILES} )
target_link_libraries( l_cameras ${pthread_LIBRARIES} )
