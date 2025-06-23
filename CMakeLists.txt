cmake_minimum_required(VERSION 3.8)
project(rakingmotion_ur5e)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(rakingmotion src/rakingmotion.cpp)
add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
ament_target_dependencies(rakingmotion 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

ament_target_dependencies(servo_keyboard_input
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  )


install(TARGETS
  rakingmotion
  servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
