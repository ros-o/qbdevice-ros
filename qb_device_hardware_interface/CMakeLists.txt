cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(qb_device_hardware_interface VERSION 3.0.0 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    hardware_interface
    qb_device_driver
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    hardware_interface
    qb_device_driver
)

# Build Settings
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/qb_device_hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
set_property(
  TARGET
    ${PROJECT_NAME}
  PROPERTY CXX_STANDARD 14
)

set_property(
  TARGET
    ${PROJECT_NAME}
  PROPERTY CXX_STANDARD_REQUIRED ON
)