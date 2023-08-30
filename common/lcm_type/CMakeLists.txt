cmake_minimum_required(VERSION 3.5)

project(lcm_type)

find_package(lcm REQUIRED)

set(FILE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lcm/*.lcm")
set(OUT_DIR "${CMAKE_BINARY_DIR}/header/${PROJECT_NAME}")
set(GEN_CMD ${CMAKE_CURRENT_SOURCE_DIR}/gen_hpp.sh ${OUT_DIR} ${FILE_DIR})

# add_custom_target(${PROJECT_NAME}
#       COMMAND ${GEN_CMD}
#       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
# )

include(ExternalProject)

ExternalProject_Add(
  ${PROJECT_NAME}
  PREFIX ${CMAKE_BINARY_DIR}/lcm_type
  URL ${CMAKE_CURRENT_SOURCE_DIR}/lcm
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  COMMAND "${GEN_CMD}"
  TIMEOUT 10
)
