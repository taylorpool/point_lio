cmake_minimum_required(VERSION 3.25)

project(point_lio
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.4 REQUIRED)
find_package(GTSAM 4.2 REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        velodyne_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf2
        tf2_ros
        rosbag
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
        rosbag
)

add_library(PCLTypes
    src/pcl_types.cpp
)
target_sources(PCLTypes
    PUBLIC
    FILE_SET
        pcl_types_headers
    TYPE
        HEADERS
    BASE_DIRS
        include
    FILES
        include/pcl_types/pcl_types.hpp
)
target_link_libraries(PCLTypes
    Eigen3::Eigen
)

add_library(PCLTypesROS1
    src/pcl_types_ros1.cpp
)
target_sources(PCLTypesROS1
    PUBLIC
    FILE_SET
        pcl_types_ros1_headers
    TYPE
        HEADERS
    BASE_DIRS
        include
    FILES
        include/pcl_types/pcl_types_ros1.hpp
)
target_include_directories(PCLTypesROS1
    PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(PCLTypesROS1
    PCLTypes
    ${catkin_LIBRARIES}
)

add_library(IVox INTERFACE)
target_sources(IVox
    INTERFACE
    FILE_SET
        ivox_headers
    TYPE
        HEADERS
    BASE_DIRS
        include
    FILES
        include/ivox/ivox3d.h
        include/ivox/eigen_types.h
        include/ivox/ivox3d_node.hpp
        include/ivox/hilbert.hpp
)
target_link_libraries(IVox
    INTERFACE
    PCLTypes
)

add_library(VelodyneDecoder
    src/velodyne_decoder.cpp
    src/vlp16.cpp
    src/vlp32c.cpp
)
target_sources(VelodyneDecoder
    PUBLIC
    FILE_SET
        velodyne_decoder_headers
    TYPE
        HEADERS
    BASE_DIRS
        include
    FILES
        include/velodyne_decoder/velodyne_decoder.hpp
        include/velodyne_decoder/vlp16.hpp
        include/velodyne_decoder/vlp32c.hpp
)
target_link_libraries(VelodyneDecoder
    PCLTypes
)

add_library(VelodyneDecoderROS1
    src/velodyne_decoder_ros.cpp
)
target_sources(VelodyneDecoderROS1
    PUBLIC
    FILE_SET
        velodyne_decoder_ros1_headers
    TYPE
        HEADERS
    BASE_DIRS
        include
    FILES
        include/velodyne_decoder/velodyne_decoder_ros.hpp
)
target_include_directories(VelodyneDecoderROS1
    PUBLIC
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

add_executable(vlp16_bag_converter
    src/vlp16_bag_converter.cpp
)
target_link_libraries(vlp16_bag_converter
    VelodyneDecoderROS1
)

add_executable(vlp32c_node
    src/vlp32c_node.cpp
)
target_link_libraries(vlp32c_node
    VelodyneDecoderROS1
)

add_library(PointLIO
    src/incremental_kd_tree.cpp
    src/ekf.cpp
    src/point_lio.cpp
)
target_sources(PointLIO
    PUBLIC
    FILE_SET
        point_lio_headers
    TYPE
        HEADERS
    BASE_DIRS
        include
    FILES
        include/point_lio/point_lio.hpp
        include/point_lio/ekf.hpp
        include/point_lio/incremental_kd_tree.hpp
)
target_link_libraries(PointLIO
    gtsam
    gtsam_unstable
    PCLTypes
    IVox
)

add_library(PointLIOROS1
    src/point_lio_ros1.cpp
)
target_sources(PointLIOROS1
    PUBLIC
    FILE_SET
        point_lio_ros1_headers
    TYPE
        HEADERS
    BASE_DIRS
        include
    FILES
        include/point_lio_ros1/point_lio_ros1.hpp
)
target_include_directories(PointLIOROS1
    PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(PointLIOROS1
    ${catkin_LIBRARIES}
    PCLTypesROS1
    PointLIO
)

add_executable(main
    src/main.cpp
)
target_link_libraries(main
    PointLIOROS1
)

add_executable(run_bag
    src/run_bag.cpp
)
target_link_libraries(run_bag
    PointLIOROS1
)

add_executable(to_point_bag
    src/to_point_bag.cpp
    src/utils_ros1.cpp
)
target_link_libraries(to_point_bag
    PointLIOROS1
)

install(
    TARGETS
        IVox
        PCLTypes
        PCLTypesROS1
        VelodyneDecoder
        VelodyneDecoderROS1
        vlp16_node
        vlp16_bag_converter
        vlp32c_node
        PointLIO
        PointLIOROS1
        main
        run_bag
    EXPORT
        PointLIOTargets
    ARCHIVE
    LIBRARY
    RUNTIME
    FILE_SET
        pcl_types_headers
    FILE_SET
        pcl_types_ros1_headers
    FILE_SET
        velodyne_decoder_headers
    FILE_SET
        velodyne_decoder_ros1_headers
    FILE_SET
        point_lio_headers
    FILE_SET
        point_lio_ros1_headers
    FILE_SET
        ivox_headers
)

install(EXPORT PointLIOTargets
    FILE "PointLIOTargets.cmake"
    NAMESPACE PointLIO::
    DESTINATION "lib/cmake/PointLIO"
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/PointLIOConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/PointLIOConfig.cmake"
    INSTALL_DESTINATION "lib/cmake/PointLIO"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/PointLIOConfigVersion.cmake"
    COMPATIBILITY ExactVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/PointLIOConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/PointLIOConfigVersion.cmake"
    DESTINATION "lib/cmake/PointLIO"
)
