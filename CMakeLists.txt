cmake_minimum_required(VERSION 3.5)

# add this options before PROJECT keyword
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

project(cyberdog_locomotion)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif()

set(CMAKE_COLOR_MAKEFILE ON)
set(BUILD_TYPE_RELEASE TRUE)
set(CMAKE_CXX_STANDARD 14)

option(BUILD_CYBERDOG2 "Build  for  cyberdog or cyberdog2" ON)
option(BUILD_SIMULATOR "Do not build simulator" ON)
option(BUILD_HARDWARE "Build cyber dog hardware driver" ON)
option(BUILD_CONTROL "Build cyber dog controller" ON)
option(ONBOARD_BUILD "Use compiler flags for computer on robot" OFF)
option(BUILD_FACTORY "Build production test tools for factory" OFF)
option(BUILD_ROS "Build cyber dog for ROS2" OFF)

if(BUILD_CYBERDOG2)
  add_compile_definitions(BUILD_CYBERDOG2=1)
endif(BUILD_CYBERDOG2)


# Find target processor type
if(CMAKE_SYSTEM_PROCESSOR MATCHES "amd64.*|x86_64.*|AMD64.*")
  set(X86_64 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686.*|i386.*|x86.*")
  set(X86 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*|arm64.*|ARM64.*)")
  set(AARCH64 1)
endif()

if(ONBOARD_BUILD)
  add_compile_definitions(ONBOARD_BUILD=1)

  # set(CMAKE_CXX_FLAGS "-O3 -no-pie -Wall -Werror -no-pie -ggdb \ -Werror
  set(CMAKE_CXX_FLAGS "-O3 -s -no-pie -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wsign-promo -Wno-format-nonliteral -Wno-strict-aliasing \
  -static-libstdc++")
  set(CMAKE_C_FLAGS "-O3 -ggdb -std=gnu99 -I. -static-libstdc++")

  # set(CMAKE_C_FLAGS "-O3 -s -std=gnu99 -I. -static-libstdc++")
  message("**** Onboard Build enabled ****")
else(ONBOARD_BUILD)
  if(CMAKE_SYSTEM_NAME MATCHES Linux) # -Werror # dismiss eigen3-related warning with -Wno-deprecated-copy
    set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall -Wno-deprecated-copy \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wno-format-nonliteral -Wsign-promo -Wno-strict-aliasing -faligned-new")
    set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
  elseif(APPLE)
    set(CMAKE_MACOSX_RPATH ON)
    set(CMAKE_CXX_FLAGS "-O3 -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wno-format-nonliteral -Wsign-promo -Wno-strict-aliasing")
    set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -I.")
    include_directories("/usr/local/include/") # lcm includes
  endif()

  message("**** Onboard Build disabled ****")
endif(ONBOARD_BUILD)

# package for ROS2 colcon build
if(BUILD_ROS)
  find_package(ament_cmake REQUIRED)
endif(BUILD_ROS)

add_subdirectory(common)

if(ONBOARD_BUILD)
  add_subdirectory(hardware)
  add_subdirectory(control)

  if (EXISTS "${CMAKE_SOURCE_DIR}/factory" AND BUILD_FACTORY)
    add_subdirectory(factory)
  endif()
else(ONBOARD_BUILD)
  if(EXISTS "${CMAKE_SOURCE_DIR}/simulator")
    add_subdirectory(simulator)
  endif()

  add_subdirectory(simbridge)
  add_subdirectory(control)
endif(ONBOARD_BUILD)

if(ONBOARD_BUILD AND BUILD_FACTORY)
  add_custom_target(version
    COMMAND ./scripts/get_git_hash.sh
    ${PROJECT_BINARY_DIR}/version.txt
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  add_custom_target(run_pack_script
    COMMAND ./scripts/pack_runnable.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  add_dependencies(run_pack_script
    version
    cyberdog_control
    manager
    ${TEST_TARGETS}
  )


  add_custom_target(final_package ALL)
  add_dependencies(final_package run_pack_script)

endif()

# for ROS2 colcon build
if(BUILD_ROS)
  ament_package()
endif(BUILD_ROS)
