cmake_minimum_required(VERSION 3.16)
project(rcllua)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcutils REQUIRED)
find_package(Lua REQUIRED)  # >= 5.3
find_package(rcllua_cmake REQUIRED)

set(LIB_NAME rclbind)

add_library(${LIB_NAME} SHARED
  src/rclbind.c
  src/utils.c
  src/context.c
  src/logger.c
  src/time.c
  src/timer.c
  src/clock.c
  src/node.c
  src/wait_set.c
)

ament_target_dependencies(${LIB_NAME}
  rcl
  rcutils
)

set_target_properties(${LIB_NAME} PROPERTIES 
  PREFIX ""
)

# Lua scripts
rcllua_cmake_install_lib(${PROJECT_NAME})

# C library
rcllua_cmake_install_clib(${LIB_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
