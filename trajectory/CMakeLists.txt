cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3_Microservice)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    /opt/ORB_SLAM3
    /opt/ORB_SLAM3/include
    /opt/ORB_SLAM3/include/CameraModels
    /opt/ORB_SLAM3/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(trajectory trajectory.cpp)

target_link_libraries(trajectory
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    /opt/ORB_SLAM3/lib/libORB_SLAM3.so
    /opt/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
    /opt/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
    -lzmq
)
