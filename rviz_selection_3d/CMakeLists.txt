cmake_minimum_required(VERSION 3.8)
project(rviz_selection_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)


include_directories(
  ${rviz_common_INCLUDE_DIRS}
  ${rviz_rendering_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include

)

add_library(${PROJECT_NAME} SHARED

  include/select_3d_tool.hpp
  src/select_3d_tool.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  rviz_rendering
  rclcpp
)

# Install target
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Export
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rviz_common
  rviz_rendering
  rclcpp
)

ament_export_include_directories(include)
pluginlib_export_plugin_description_file(rviz_common select_3d_tool_plugin_description.xml)

ament_package()
