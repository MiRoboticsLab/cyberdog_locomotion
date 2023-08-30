add_executable(cyberdog_control main.cpp sub_main.cpp)
target_include_directories(cyberdog_control PUBLIC  "./")

if(ONBOARD_BUILD)
  target_include_directories(cyberdog_control PUBLIC  "${CMAKE_SOURCE_DIR}/hardware/include")
  target_link_libraries(cyberdog_control hardware control)
else(ONBOARD_BUILD)
  target_include_directories(cyberdog_control PUBLIC  "${CMAKE_SOURCE_DIR}/simbridge/include")
  target_link_libraries(cyberdog_control simbridge control)
endif(ONBOARD_BUILD)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(cyberdog_control rt)
endif()

# install for ROS2 colcon build
if(BUILD_ROS)
  install(TARGETS cyberdog_control RUNTIME DESTINATION lib/cyberdog_locomotion)
endif(BUILD_ROS)
