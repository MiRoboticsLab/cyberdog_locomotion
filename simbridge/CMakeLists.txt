cmake_minimum_required(VERSION 3.5)
project(simbridge)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)


set(CMAKE_CXX_STANDARD 14)
set(BUILD_TYPE_RELEASE TRUE)

SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH_1})

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 OLD)
endif()


find_package(Eigen3 3.3.7 EXACT REQUIRED)
find_package(lcm REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

include_directories("include/")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_SOURCE_DIR}/common/third_party/ParamHandler")
include_directories("${CMAKE_SOURCE_DIR}/common/include")
include_directories("${CMAKE_SOURCE_DIR}/control/include")
include_directories(${CMAKE_BINARY_DIR})

file(GLOB_RECURSE sources src/*.cpp)

set(OpenGL_GL_PREFERENCE LEGACY)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

add_library(simbridge SHARED ${sources} ${tst_hdr_moc})
target_link_libraries(simbridge pthread lcm param_handler common control ${OPENGL_LIBRARIES})
