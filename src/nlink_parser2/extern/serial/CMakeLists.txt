cmake_minimum_required(VERSION 3.15)
project(serial LANGUAGES CXX)

add_library(asio INTERFACE)

target_include_directories(asio INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/../asio/asio/include
)

target_compile_definitions(asio INTERFACE
  ASIO_STANDALONE
)

add_library(serial STATIC)

set_target_properties(serial PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(serial PRIVATE
    -Wno-unused-but-set-variable    
    -Wno-sign-compare              
  )
endif()

target_sources(serial PRIVATE
  src/serial_port.cpp
)

target_include_directories(serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(serial PUBLIC
  asio
)

target_compile_features(serial PUBLIC cxx_std_17)
