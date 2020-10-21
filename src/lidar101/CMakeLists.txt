cmake_minimum_required(VERSION 3.5)
project(lidar101)

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

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        )

add_library(${PROJECT_NAME}_lib
        src/lidar101.cpp
        )

target_link_libraries(${PROJECT_NAME}_lib
        ${PCL_LIBRARIES}
        )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

add_executable(${PROJECT_NAME}
        src/lidar101_node.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
