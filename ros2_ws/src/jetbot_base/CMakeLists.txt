cmake_minimum_required(VERSION 3.5)
project(jetbot_base)

# =========================
# Language / warnings
# =========================
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Wheel odom extras (add these to package.xml too)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(sensor_msgs REQUIRED)  # Uncomment if publishing JointState
find_package(diagnostic_msgs REQUIRED)

# =========================
# Include dirs
# =========================
include_directories(include)

# =========================
# Motor driver library + node
# =========================
add_library(jetbot_motor_driver
  src/jetbot_motor_driver.cpp
)
target_include_directories(jetbot_motor_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(jetbot_motor_driver
  rclcpp
)

add_executable(jetbot_motor_driver_node
  src/jetbot_motor_driver_node.cpp
)
target_link_libraries(jetbot_motor_driver_node
  jetbot_motor_driver
)
ament_target_dependencies(jetbot_motor_driver_node
  rclcpp
  geometry_msgs
)

# =========================
# Wheel odom library + node
# =========================
add_library(wheel_odom_node
  src/wheel_odom_node.cpp
)
target_include_directories(wheel_odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(wheel_odom_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  # sensor_msgs  # Uncomment if used
  diagnostic_msgs
)

add_executable(jetbot_wheel_odom
  src/wheel_odom_main.cpp
)
target_link_libraries(jetbot_wheel_odom
  wheel_odom_node
)
ament_target_dependencies(jetbot_wheel_odom
  rclcpp
  diagnostic_msgs
)

# =========================
# Install
# =========================
install(TARGETS
  jetbot_motor_driver
  wheel_odom_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  jetbot_motor_driver_node
  jetbot_wheel_odom
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Export (optional but nice)
# =========================
ament_export_include_directories(include)
ament_export_libraries(jetbot_motor_driver wheel_odom_node)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

# =========================
# Lint / tests
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
