cmake_minimum_required(VERSION 3.0.2)
project(kitti_pkg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")


#include_directories(
#        # include
#        ${catkin_INCLUDE_DIRS}
#)

find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rospy
    cv_bridge
    image_transport

    pcl_ros
    pcl_conversions

    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)

catkin_package(
        INCLUDE_DIRS include
        DEPENDS PCL
)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS regex thread system filesystem)
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)

include_directories(
        include
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)


message(${PROJECT_SOURCE_DIR})
aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_DIR)

add_library(${PROJECT_NAME} SHARED
        ${SRC_DIR}
        )


target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBS}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )


link_directories(
        include
        ${OpenCV_LIBRARY_DIRS}
        ${PCL_LIBRARY_DIRS}
)


add_executable(kitti2pcd nodecpp/kitti2pcd.cpp)
target_link_libraries(kitti2pcd ${PROJECT_NAME})


add_executable(kitti_pose nodecpp/kitti_pose.cpp)
target_link_libraries(kitti_pose ${PROJECT_NAME})


