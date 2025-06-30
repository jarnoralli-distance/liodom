cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0057 NEW)

project(liodom LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add cmake modules from the cmake-directory
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Non-ROS2 packages
find_package(PCL 1.13 REQUIRED)
find_package(EIGEN3)
find_package(Ceres 2.0 REQUIRED)
find_package(OpenMP REQUIRED)

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED )
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)


###########
## Build ##
###########

#### Laser Odometry ####
add_executable(liodom_node
    src/liodom_node.cc
    src/shared_data.cc
    src/stats.cc
    src/params.cc
    src/feature_extractor.cc
    src/laser_odometry.cc
)

target_include_directories(liodom_node
    PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(
    liodom_node
    PRIVATE
        rclcpp::rclcpp
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        sensor_msgs::sensor_msgs_library
        
        tf2::tf2
        tf2_ros::tf2_ros
        pcl_conversions::pcl_conversions
        PCL::PCL
        Ceres::ceres
        OpenMP::OpenMP_CXX
)

#### Laser Mapping ####
add_executable(liodom_mapping
    src/map.cc
    src/liodom_mapping_node.cc
)

target_include_directories(liodom_mapping
    PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(liodom_mapping
    PUBLIC
        rclcpp::rclcpp
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        sensor_msgs::sensor_msgs_library
        tf2::tf2
        tf2_ros::tf2_ros
        tf2_eigen::tf2_eigen
        pcl_conversions::pcl_conversions
        PCL::PCL
        OpenMP::OpenMP_CXX
)

install(TARGETS
    liodom_node
    liodom_mapping
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch
    rviz 
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
