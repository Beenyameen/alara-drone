cmake_minimum_required(VERSION 3.0)
project(reconstruction)
find_package(RTABMap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    /opt/ros/noetic/include
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    /opt/ros/noetic/lib
    ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(reconstruction reconstruction.cpp)
target_link_libraries(reconstruction 
    ${RTABMap_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
    -lzmq
    -lg2o_core
    -lg2o_types_slam2d
    -lg2o_types_slam3d
    -lg2o_types_sba
    -lg2o_stuff
    -lg2o_csparse_extension
    -pthread
)
