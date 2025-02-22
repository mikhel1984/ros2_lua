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

include_directories(include ${LUA_INCLUDE_DIR})

# MESSAGES (shared library)

set(LIB_NAME sequence)

add_library(${LIB_NAME} SHARED
  src/luacommon.c
  src/float_sequence.c
  src/int_sequence.c
  src/boolean_sequence.c
  src/string_sequence.c
  src/utility.c
)
ament_target_dependencies(${LIB_NAME} rosidl_runtime_c)

set_target_properties(${LIB_NAME} PROPERTIES
  PREFIX ""
)

# export header
install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories("include")

# library access
rcllua_cmake_install_clib(${PROJECT_NAME} ${LIB_NAME})

# FUNCTIONS (static library) 

set(UTILITY_LIB rosidl_utility)

add_library(${UTILITY_LIB}
  src/utility.c
)
ament_target_dependencies(${UTILITY_LIB} rosidl_runtime_c)

install(
  TARGETS ${UTILITY_LIB}
  EXPORT ${UTILITY_LIB}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(${UTILITY_LIB} HAS_LIBRARY_TARGET)

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
