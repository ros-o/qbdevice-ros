cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(qb_device_hardware_interface VERSION 1.0.2 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    hardware_interface
    joint_limits_interface
    transmission_interface
    urdf
    qb_device_msgs
    qb_device_srvs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    lib${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    hardware_interface
    joint_limits_interface
    transmission_interface
    urdf
    qb_device_msgs
    qb_device_srvs
)

# Exported libraries
add_library(lib${PROJECT_NAME}
  src/qb_device_hardware_interface.cpp
)

add_dependencies(lib${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

set_target_properties(lib${PROJECT_NAME}
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}  # avoid "liblib" name
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

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # explicit gmock build FIXME: outside sourcing
  include_directories(/usr/include/gmock /usr/src/gmock)
  find_library(GMOCK_LIBRARIES libgmock)
  if(NOT GMOCK_LIBRARIES)
    add_library(libgmock /usr/src/gmock/src/gmock-all.cc)
  endif()

  catkin_add_gtest(${PROJECT_NAME}_unit_tests
    src/test/unit_tests.cpp
  )

  target_link_libraries(${PROJECT_NAME}_unit_tests
    libgmock
    lib${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  set_property(
    TARGET
      ${PROJECT_NAME}_unit_tests
    PROPERTY CXX_STANDARD 14
  )

  set_property(
    TARGET
      ${PROJECT_NAME}_unit_tests
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif()