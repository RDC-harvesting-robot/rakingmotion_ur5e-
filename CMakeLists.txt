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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(executor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# --- rakingmotion ---
add_executable(rakingmotion src/rakingmotion.cpp)
ament_target_dependencies(rakingmotion 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

# --- servo_keyboard_input ---
add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
ament_target_dependencies(servo_keyboard_input
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

# --- auto_rakingmotion ---
add_executable(auto_rakingmotion src/auto_rakingmotion.cpp)
ament_target_dependencies(auto_rakingmotion
  rclcpp        # <-- 修正: 必須
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  sensor_msgs   # <-- 修正: cv_bridge に必要
  # OpenCV は ament パッケージではないため、ここから削除
  )
# cv_bridge が OpenCV ライブラリをリンクします。
# もしリンクエラーが出たら、以下を追加してください:
# target_link_libraries(auto_rakingmotion PUBLIC ${OpenCV_LIBRARIES})


# --- forward_rakingmotion ---
add_executable(forward_rakingmotion src/forward_rakingmotion.cpp)
ament_target_dependencies(forward_rakingmotion
  rclcpp        # <-- 修正: 必須
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  executor_msgs 
  std_srvs
  )

# --- return_rakingmotion ---
add_executable(return_rakingmotion src/return_rakingmotion.cpp)
ament_target_dependencies(return_rakingmotion
  rclcpp        # <-- 修正: 必須
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  executor_msgs  
  std_srvs
  )

# --- plan_cliant ---
add_executable(plan_cliant src/plan_cliant.cpp)
ament_target_dependencies(plan_cliant
  rclcpp        # <-- 修正: 必須
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  executor_msgs
  )

# --- Installation (修正: 1つに統合) ---
install(TARGETS
  rakingmotion
  servo_keyboard_input
  auto_rakingmotion
  return_rakingmotion
  forward_rakingmotion
  plan_cliant
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()