cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

project(control)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
set(BUILD_TYPE_RELEASE TRUE)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 3.3.7 EXACT REQUIRED)
FIND_PACKAGE(lcm REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories("./")
include_directories("include")
include_directories("include/control")
include_directories("third_party")
include_directories(SYSTEM "third_party/qpOASES/include")
include_directories(${CMAKE_BINARY_DIR})
include_directories("${CMAKE_SOURCE_DIR}/common/third_party")
include_directories("${CMAKE_SOURCE_DIR}/common/third_party/ParamHandler")
include_directories("${CMAKE_SOURCE_DIR}/common/include")


# for ROS2 colcon build
if(BUILD_ROS)
  include_directories("/opt/ros/galactic/opt/yaml_cpp_vendor/include")
endif(BUILD_ROS)


file(GLOB_RECURSE sources "src/*.cpp")

# generate control library
add_library(control SHARED ${sources})
target_link_libraries(control
  pthread
  lcm
  param_handler
  qpOASES
  Goldfarb_Optimizer
  common
)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
target_link_libraries(control rt)
endif()

add_subdirectory(third_party)
add_subdirectory(user)
