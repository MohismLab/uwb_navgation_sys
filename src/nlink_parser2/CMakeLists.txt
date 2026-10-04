cmake_minimum_required(VERSION 3.8)
project(nlink_parser2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LinktrackTag.msg"
  "msg/LinktrackNode0.msg"
  "msg/LinktrackNode1.msg"
  "msg/LinktrackNode2.msg"
  "msg/LinktrackNode4Anchor.msg"
  "msg/LinktrackNode5.msg"
  "msg/LinktrackNode6.msg"
  "msg/LinktrackNode7.msg"
  
  "msg/LinktrackNode4Tag.msg"      
  "msg/LinktrackTagframe0.msg"
  
  "msg/LinktrackNodeframe0.msg"    
  "msg/LinktrackNodeframe1.msg"    
  "msg/LinktrackNodeframe2.msg"    
  "msg/LinktrackNodeframe3.msg"    
  "msg/LinktrackNodeframe5.msg"    
  "msg/LinktrackNodeframe6.msg"    
  "msg/LinktrackNodeframe7.msg"    
  
  "msg/LinktrackAoaNode0.msg"    
  "msg/LinktrackAoaNodeframe0.msg"    

  "msg/LinktrackNodeframe4.msg"    
  "msg/LinktrackAnchorframe0.msg"  
  
  "msg/TofsenseFrame0.msg"
  "msg/TofsenseMFrame0Pixel.msg"
  "msg/TofsenseMFrame0.msg"        
  "msg/TofsenseCascade.msg"        
  
  DEPENDENCIES builtin_interfaces std_msgs
)

add_subdirectory(extern/serial)

add_subdirectory(src/nlink_utils)

add_executable(tofsense_node
  src/tofsense_node/src/tofsenseinit.cpp
  src/tofsense_node/src/main.cpp  
)

add_executable(tofsensem_node
  src/tofsensem_node/src/tofsenseminit.cpp  
  src/tofsensem_node/src/main.cpp         
)

add_executable(linktrack_node
  src/linktrack_node/src/linktrackinit.cpp 
  src/linktrack_node/src/main.cpp          
)

add_executable(linktrack_aoa_node
  src/linktrack_aoa_node/src/linktrackaoainit.cpp  
  src/linktrack_aoa_node/src/main.cpp              
)

target_include_directories(tofsense_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  extern/serial/include
  src/nlink_utils/include
  src/tofsense_node/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_include_directories(tofsensem_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  extern/serial/include
  src/nlink_utils/include
  src/tofsensem_node/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_include_directories(linktrack_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  extern/serial/include
  src/nlink_utils/include
  src/linktrack_node/include  
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_include_directories(linktrack_aoa_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  extern/serial/include
  src/nlink_utils/include
  src/linktrack_aoa_node/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

foreach(target_name tofsense_node tofsensem_node linktrack_node linktrack_aoa_node)
  ament_target_dependencies(${target_name}
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    builtin_interfaces
  )

  target_link_libraries(${target_name}
    serial          
    nlink_utils   
    asio 
  )

  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME}
    rosidl_typesupport_cpp
  )
  target_link_libraries(${target_name} "${cpp_typesupport_target}")
endforeach()

install(TARGETS serial nlink_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS tofsense_node tofsensem_node linktrack_node linktrack_aoa_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch msg config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.dae"
)

install(PROGRAMS scripts/uwb_ekf_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(serial nlink_utils)
ament_export_dependencies(rclcpp builtin_interfaces std_msgs rosidl_default_generators)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
