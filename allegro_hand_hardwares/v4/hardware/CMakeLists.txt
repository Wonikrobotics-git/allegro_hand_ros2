cmake_minimum_required(VERSION 3.8)
project(allegro_hand_v4_hardware)

# Set the default build type to Release if it's not specified.
# This enables optimizations and disables debug information by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting build type to 'Release'")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(spdlog REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(allegro_hand_io REQUIRED)
find_package(allegro_hand_utility REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  hardware_interface 
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  spdlog 
  diagnostic_updater
  allegro_hand_io
  allegro_hand_utility
)

add_library(
  allegro_hand_v4_hardware 
  SHARED  
  ./src/v4_hardware_interface.cpp
  ./src/generic_system.cpp
  ./src/v4_spec.cpp
  ./src/v4_sdk.cpp
)

# target_compile_definitions(allegro_hand_v4_hardware PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

add_executable(interface_test ./src/v4_interface_test.cpp)

target_compile_definitions(interface_test PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

target_link_libraries(interface_test spdlog::spdlog )

#add headfile source
target_include_directories(allegro_hand_v4_hardware PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/allegro_hand_v4_hardware>
)
target_link_libraries(allegro_hand_v4_hardware)

#add dependencies
ament_target_dependencies(interface_test ${dependencies}) 
ament_target_dependencies(allegro_hand_v4_hardware ${dependencies})


#export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface allegro_hand_v4_hardware.xml)

#interface test 
install(TARGETS interface_test
  DESTINATION lib/${PROJECT_NAME}
)

#ros2_control hardware
install(TARGETS allegro_hand_v4_hardware
  EXPORT allegro_hand_v4_hardware
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#export
ament_export_targets(allegro_hand_v4_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle)

ament_package()
