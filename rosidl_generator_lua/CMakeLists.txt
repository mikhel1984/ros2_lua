cmake_minimum_required(VERSION 3.8)
project(rosidl_generator_lua)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
#find_package(ament_cmake_export_assemblies REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_generator_c)

ament_index_register_resource("rosidl_generator_packages")
ament_python_install_package(${PROJECT_NAME})

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

install(
  PROGRAMS bin/rosidl_generator_lua
  DESTINATION lib/rosidl_generator_lua
)

install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS 
    "cmake/rosidl_generator_lua_get_typesupports.cmake"
    "cmake/register_lua.cmake"
    "rosidl_generator_lua-extras.cmake.in"
)
