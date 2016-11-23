cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(qb_device_driver VERSION 3.0.0 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
)

include_directories(include
  lib/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
    lib/src
  LIBRARIES
    lib${PROJECT_NAME}
)

# Exported libraries
add_library(lib${PROJECT_NAME}
  lib/src/qbmove_communications.cpp
)

# C++ Settings
set_property(
  TARGET
    lib${PROJECT_NAME}
  PROPERTY CXX_STANDARD 14
)

set_property(
  TARGET
    lib${PROJECT_NAME}
  PROPERTY CXX_STANDARD_REQUIRED ON
)