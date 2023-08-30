cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

project(hardware)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
set(BUILD_TYPE_RELEASE TRUE)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 3.3.7 EXACT REQUIRED)
find_package(lcm REQUIRED)

file(GLOB_RECURSE sources src/*.c  src/*.cpp)

# generate hardware library
add_library(hardware SHARED ${sources})
target_include_directories(hardware PUBLIC
                    "include"
                    "include/hardware"
                   "${CMAKE_SOURCE_DIR}/common/third_party/ParamHandler"
                   "${CMAKE_SOURCE_DIR}/common/include"
                   "${CMAKE_SOURCE_DIR}/control/include"
                    ${EIGEN3_INCLUDE_DIRS}
                   "${CMAKE_BINARY_DIR}"
)
target_link_libraries(hardware
  pthread
  lcm
  param_handler
  common
  control
)
if(CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(hardware rt)
endif()

FILE(GLOB _manager_sources "${PROJECT_SOURCE_DIR}/src/manager/*.cpp")
add_library(manager_sources ${_manager_sources})
add_dependencies(manager_sources lcm_type)

include_directories(/usr/local/include)
include_directories("include/manager")
include_directories("${CMAKE_SOURCE_DIR}/common/third_party/rapidjson")

target_include_directories(manager_sources PUBLIC
  "include/hardware"
  "include/manager"
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}
  "${CMAKE_SOURCE_DIR}/common/third_party/ParamHandler"
  "${CMAKE_SOURCE_DIR}/common/third_party/rapidjson"
  "${CMAKE_SOURCE_DIR}/common/include"
  ${RapidJSON_INCLUDE_DIR}
)

if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  target_include_directories(manager_sources PUBLIC ${RAPIDJSON_INCLUDE_DIRS})
endif()

add_executable(manager src/manager.cpp)
target_link_libraries(manager manager_sources common hardware lcm pthread)
target_link_libraries(manager PUBLIC ${RAPIDJSON_INCLUDE_DIRS})

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(manager rt)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)
