cmake_minimum_required(VERSION 3.5)
project(perse_webots)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG aa8d4e4750ec9fe9f8cc680eb90f1b15955c817e
)
FetchContent_MakeAvailable(yaml-cpp)

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver perse_driver.xml)

# PerseDriver library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/PerseDriver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
target_link_libraries(
  ${PROJECT_NAME}
  yaml-cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
  nav_msgs
  std_msgs
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  meshes
  protos
  config
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
