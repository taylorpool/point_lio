cmake_minimum_required(VERSION 3.16)

project(point_lio
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.4 REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        velodyne_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf2
        tf2_ros
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        PCLTypes
        PCLTypesROS1
        VelodyneDecoder
        VelodyneDecoderROS1
    CATKIN_DEPENDS
        roscpp
        velodyne_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf2
        tf2_ros
)

add_library(PCLTypes
    src/pcl_types.cpp
)
target_include_directories(PCLTypes
    PUBLIC
    include
)
target_link_libraries(PCLTypes
    Eigen3::Eigen
)

add_library(PCLTypesROS1
    src/pcl_types_ros1.cpp
)
target_include_directories(PCLTypesROS1
    PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(PCLTypesROS1
    PCLTypes
    ${catkin_LIBRARIES}
)

add_library(VelodyneDecoder
    src/velodyne_decoder.cpp
    src/vlp16.cpp
    src/vlp32c.cpp
)
target_include_directories(VelodyneDecoder
    PUBLIC
    include
)
target_link_libraries(VelodyneDecoder
    PCLTypes
)

add_library(VelodyneDecoderROS1
    src/velodyne_decoder_ros.cpp
)
target_include_directories(VelodyneDecoderROS1
    PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(VelodyneDecoderROS1
    VelodyneDecoder
    PCLTypesROS1
    ${catkin_LIBRARIES}
)

add_executable(vlp16_node
    src/vlp16_node.cpp
)
target_link_libraries(vlp16_node
    VelodyneDecoderROS1
)

add_executable(vlp32c_node
    src/vlp32c_node.cpp
)
target_link_libraries(vlp32c_node
    VelodyneDecoderROS1
)

add_executable(main
    src/main.cpp
)
target_include_directories(main
    PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(main
    PCLTypesROS1
    ${catkin_LIBRARIES}
)
