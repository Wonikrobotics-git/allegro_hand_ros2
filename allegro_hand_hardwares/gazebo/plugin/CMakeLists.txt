cmake_minimum_required(VERSION 3.5)
project(allegro_hand_gazebo_plugin)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(gz_ros2_control REQUIRED)

find_package(spdlog REQUIRED)

set(GZ_PLUGIN)
set(GZ_SIM)

if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo harmonic")
  find_package(gz-plugin2 REQUIRED)
  set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
  set(GZ_PLUGIN gz-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM gz-sim${GZ_SIM_VER}::core)
  add_definitions(-DGZ_HEADERS)
else()
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo fortress")
  find_package(ignition-plugin1 REQUIRED)
  set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
  set(GZ_PLUGIN ignition-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM ignition-gazebo${GZ_SIM_VER}::core)
endif()

include_directories(include)

add_library(ah_gazebo_plugin SHARED
  src/allegro_hand_system.cpp
)
ament_target_dependencies(ah_gazebo_plugin
  rclcpp_lifecycle
  hardware_interface
  rclcpp
  gz_ros2_control
)
target_link_libraries(ah_gazebo_plugin
  ${GZ_SIM}

  spdlog::spdlog
)

target_compile_definitions(ah_gazebo_plugin PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

## Install
install(TARGETS
  ah_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)
 
# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include) 

ament_export_libraries(ah_gazebo_plugin)
pluginlib_export_plugin_description_file(gz_ros2_control allegro_hand_gazebo_plugin.xml)

# Setup the project
ament_package()
