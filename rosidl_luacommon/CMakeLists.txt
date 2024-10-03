cmake_minimum_required(VERSION 3.8)
project(rosidl_luacommon)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(Lua REQUIRED)  # >= 5.3
find_package(rcllua_cmake REQUIRED)

include_directories(include)

set(LIB_NAME rosidl_luacommon)

add_library(${LIB_NAME} SHARED
  src/luacommon.c
  src/float_sequence.c
  src/int_sequence.c
  src/boolean_sequence.c
  src/string_sequence.c
)

ament_target_dependencies(${LIB_NAME}
  rosidl_runtime_c
)

set_target_properties(${LIB_NAME} PROPERTIES 
  PREFIX ""
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_install_lua_clib(${LIB_NAME})

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
