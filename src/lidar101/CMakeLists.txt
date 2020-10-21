cmake_minimum_required(VERSION 3.5)
project(lidar101) # this must be the same as the name in the package.xml file

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)

# informs cmake of the directories we are including header
#  files from
include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        )

# adds a library target containing the source code from src/lidar101.cpp
add_library(${PROJECT_NAME}_lib
        src/lidar101.cpp
        )

# links the pcl libraries to our library
target_link_libraries(${PROJECT_NAME}_lib
        ${PCL_LIBRARIES}
        )

# links the pcl directories and adds the definitions
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# links the dependencies of our library to our library
ament_target_dependencies(${PROJECT_NAME}_lib
        rclcpp
        sensor_msgs
        tf2
        tf2_ros
        tf2_sensor_msgs
        Eigen3
        pcl_conversions
        pcl_ros
        )

# adds a executable target from our src/lidar101_node.cpp file
add_executable(${PROJECT_NAME}
        src/lidar101_node.cpp
        )

# links our library to our executable
target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_lib)

# installs all targets and required files (launch, config etc.) to the required
# destinations for ROS2 to access.
install(TARGETS ${PROJECT_NAME}_lib DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
        )

# for testing which is not included in the package
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# required line for ROS2 and ament
ament_package()
