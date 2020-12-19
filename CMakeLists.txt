cmake_minimum_required(VERSION 3.0.2)
project(kitti_pkg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



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


find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS regex thread system filesystem)
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)

catkin_package(
        INCLUDE_DIRS include
        DEPENDS PCL
)

aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_DIR)
add_library(${PROJECT_NAME} SHARED
        ${SRC_DIR}
        )

message(${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        )


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

link_directories(
        include
        ${OpenCV_LIBRARY_DIRS}
        ${PCL_LIBRARY_DIRS}
)
#/home/qsy-5208/Documents/LeGO/LeGO-LOAM-kitti/catkin_ws/src/kitti_pkg/src
add_executable(kitti2pcd src/kitti2pcd.cpp)
target_link_libraries(kitti2pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}) # ${Boost_LIBRARIES}
target_link_libraries(kitti2pcd ${PROJECT_NAME})

