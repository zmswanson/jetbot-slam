cmake_minimum_required(VERSION 3.5)
project(jetbot_base)

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
find_package(geometry_msgs REQUIRED)

# Headers
include_directories(include)

add_library(jetbot_motor_driver
  src/jetbot_motor_driver.cpp
)
target_include_directories(jetbot_motor_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(jetbot_motor_driver_node
  src/jetbot_motor_driver_node.cpp
)
target_link_libraries(jetbot_motor_driver_node jetbot_motor_driver)
ament_target_dependencies(jetbot_motor_driver_node rclcpp geometry_msgs)

install(TARGETS
  jetbot_motor_driver
  jetbot_motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
