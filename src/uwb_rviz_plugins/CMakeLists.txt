cmake_minimum_required(VERSION 3.8)
project(uwb_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/uwb_nav_panel.cpp
  src/uwb_nav_panel.hpp
)
target_include_directories(${PROJECT_NAME} PRIVATE src)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME} pluginlib rclcpp rviz_common std_msgs)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
