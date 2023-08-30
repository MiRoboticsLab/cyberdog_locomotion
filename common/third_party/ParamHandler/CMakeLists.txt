


option(PARAM_TEST "Build test programe for ParamHandler")


# select version of yaml-cpp for different build methods
if(BUILD_ROS)
  find_package(yaml_cpp_vendor REQUIRED)
  set(YAML_CPP_INCLUDE_DIRS "/opt/ros/galactic/opt/yaml_cpp_vendor/include")
  set(YAML_CPP_LIBRARIES "/opt/ros/galactic/opt/yaml_cpp_vendor/lib/libyaml-cpp.so")
  include_directories(${YAML_CPP_INCLUDE_DIRS})
elseif(NOT BUILD_ROS)
  find_package(yaml-cpp 0.6.3 EXACT REQUIRED)
endif(BUILD_ROS)



###
### Library
###
add_library(param_handler SHARED
  ParamHandler.hpp
  ParamHandler.cpp
)

target_include_directories(param_handler PUBLIC
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(param_handler
  ${YAML_CPP_LIBRARIES}
)

if (PARAM_TEST)
  add_executable(test_param
    test/test_param_main.cpp
  )

  target_link_libraries(test_param
    param_handler
  )
endif()
