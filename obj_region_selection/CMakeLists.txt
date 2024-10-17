cmake_minimum_required(VERSION 3.5)
project(obj_region_selection)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rviz_selection_3d REQUIRED)
find_package(glm REQUIRED)

# Set up include directories
set(INCLUDE_DIRS
  include
  include/${PROJECT_NAME}
)

foreach(dir IN LISTS INCLUDE_DIRS)
  include_directories(${dir})
endforeach()

# Include directories
include_directories(
  ${rviz_common_INCLUDE_DIRS}
  ${rviz_rendering_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_selection_3d_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Main library
add_library(${PROJECT_NAME}_lib SHARED
  "include/ogre_mesh_plugin.hpp"
  "include/ogre_mesh_plugin.cpp"
  "include/obj_region_selection_display.hpp"
  "src/obj_region_selection_display.cpp"
)

# Interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "interfaces/msg/SendMeshInfo.msg"
  "interfaces/srv/SendMeshTexture.srv"
  "interfaces/srv/ToggleService.srv"
  DEPENDENCIES
  geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  "${cpp_typesupport_target}"
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rviz_common
  rviz_rendering
  rclcpp
  pluginlib
  rviz_selection_3d
)

# Install target
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION lib
)

install(
  DIRECTORY test_parts
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(rviz_common obj_region_selection_plugin_description.xml)

ament_package()