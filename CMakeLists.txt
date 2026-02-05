cmake_minimum_required(VERSION 3.12.4)
project(direct_lidar_inertial_odometry)

# ---------------------------
# Find Packages
# ---------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros2can_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)  
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

# ---------------------------
# Project Settings
# ---------------------------
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -flto")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -march=native -flto")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${ros2can_msgs_INCLUDE_DIRS}
)

# ---------------------------
# CPUID Check
# ---------------------------
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)

# ---------------------------
# Odometry Node
# ---------------------------
add_executable(dlio_odom_node src/dlio/odom_node.cc src/dlio/odom.cc)

# Optional: CPUID macro
if(HAS_CPUID)
    target_compile_definitions(dlio_odom_node PRIVATE HAS_CPUID)
endif()

# Dependencies
ament_target_dependencies(dlio_odom_node 
    "tf2_ros" "pcl_ros" "nav_msgs" "pcl_conversions" "rclcpp" "sensor_msgs" "geometry_msgs" "ros2can_msgs"
)

# Libraries
target_link_libraries(dlio_odom_node 
    ${PCL_LIBRARIES} 
    Threads::Threads 
    OpenMP::OpenMP_CXX
)

# ---------------------------
# Mapping Node
# ---------------------------
add_executable(dlio_map_node src/dlio/map_node.cc src/dlio/map.cc)

ament_target_dependencies(dlio_map_node 
    "tf2_ros" "pcl_ros" "pcl_conversions" "rclcpp"
)

target_link_libraries(dlio_map_node 
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

# ---------------------------
# Exports & Install
# ---------------------------
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros)

install(TARGETS dlio_odom_node dlio_map_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY cfg launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
