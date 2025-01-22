cmake_minimum_required(VERSION 3.8)
project(rcllua_std_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_cmake REQUIRED)

find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rmw REQUIRED)

set(Python3_FIND_UNVERSIONED_NAMES FIRST)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

find_package(rcllua_cmake REQUIRED)
find_package(rosidl_luacommon REQUIRED)
find_package(rosidl_generator_lua REQUIRED)

include(cmake/rcllua_generate_interfaces.cmake)

# Generate lua libraries
rcllua_generate_interfaces(builtin_interfaces)
rcllua_generate_interfaces(std_msgs builtin_interfaces)

ament_package()
