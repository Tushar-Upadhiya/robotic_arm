cmake_minimum_required(VERSION 3.8)
project(robotic_arm)  # <--- THIS IS WHAT WAS MISSING

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(arm_node src/arm_node.cpp)

ament_target_dependencies(arm_node rclcpp sensor_msgs)

install(TARGETS arm_node DESTINATION lib/${PROJECT_NAME})

ament_package() # <--- MUST BE THE LAST LINE