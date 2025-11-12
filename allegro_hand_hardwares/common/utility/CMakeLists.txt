cmake_minimum_required(VERSION 3.16)
project(allegro_hand_utility LANGUAGES CXX)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting build type to 'Release'")
endif()

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)

# --- Build Library ---
set(LIB_SOURCES
  src/utility.cpp
  src/ros_utility.cpp
)

add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies using imported targets
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  spdlog
  Boost
)

# --- Test Executable ---
add_executable(test_library test/test_library.cpp)
target_link_libraries(test_library PUBLIC ${PROJECT_NAME})

# --- Installation ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS test_library
  DESTINATION lib/${PROJECT_NAME}
)

# --- Exporting ---
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp hardware_interface spdlog Boost)
ament_package()

