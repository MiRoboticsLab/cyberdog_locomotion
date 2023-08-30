
if(ONBOARD_BUILD)
  SET (THIS_COM "../" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
else(ONBOARD_BUILD)
  SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
endif(ONBOARD_BUILD)

find_package(Eigen3 3.3.7 EXACT REQUIRED)
find_package(lcm REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories("include")
include_directories("${CMAKE_SOURCE_DIR}/common/third_party/ParamHandler")
include_directories("${CMAKE_SOURCE_DIR}/common/third_party/rapidjson")
include_directories("${CMAKE_SOURCE_DIR}/common/third_party/cpptoml/include")
include_directories(${CMAKE_BINARY_DIR})

# for ROS2 colcon build
if(BUILD_ROS)
  include_directories("/opt/ros/galactic/opt/yaml_cpp_vendor/include")
endif(BUILD_ROS)
                      
file(GLOB_RECURSE sources "src/*.cpp")

# generate common library
add_library(common SHARED ${sources})
add_dependencies(common lcm_type)
target_link_libraries(common
  pthread
  lcm
  param_handler
  cpptoml
)

add_subdirectory(lcm_type)
add_subdirectory(third_party/ParamHandler)
add_subdirectory(third_party/cpptoml)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()


install(TARGETS common param_handler
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
