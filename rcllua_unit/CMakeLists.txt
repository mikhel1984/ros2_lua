cmake_minimum_required(VERSION 3.8)
project(rcllua_unit)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcllua_cmake REQUIRED)

# Lua scripts
rcllua_cmake_install_lib(${PROJECT_NAME})

# CMake function
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  ${CMAKE_CURRENT_LIST_DIR}/cmake/rcllua_unit_tests.cmake
)

ament_package()
