cmake_minimum_required(VERSION 3.8)
project(rov_sonar_driver)

# 默认 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rov_sonar_interface REQUIRED)

add_executable(sonar_omni_driver src/sonar_omni_driver.cpp)
target_include_directories(sonar_omni_driver PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)
ament_target_dependencies(sonar_omni_driver
  rclcpp
  sensor_msgs
  rov_sonar_interface
)

install(TARGETS sonar_omni_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
