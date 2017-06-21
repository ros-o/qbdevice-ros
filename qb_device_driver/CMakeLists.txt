cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(qb_device_driver VERSION 1.0.3 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    qb_device_srvs
)

include_directories(include
  api/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
    api/src
  CATKIN_DEPENDS
    roscpp
    qb_device_srvs
)

# Build Settings
add_executable(qb_device_communication_handler
  src/main.cpp
  src/qb_device_communication_handler.cpp
  api/src/qbmove_communications.cpp
)

target_link_libraries(qb_device_communication_handler
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(qb_device_communication_handler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
set_property(
  TARGET
    qb_device_communication_handler
  PROPERTY CXX_STANDARD 14
)

set_property(
  TARGET
    qb_device_communication_handler
  PROPERTY CXX_STANDARD_REQUIRED ON
)

# Installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)