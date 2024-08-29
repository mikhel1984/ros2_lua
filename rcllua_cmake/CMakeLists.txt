cmake_minimum_required(VERSION 3.8)
project(rcllua_cmake)

# find dependencies
find_package(ament_cmake REQUIRED)

# hooks
install(
  DIRECTORY templates
  DESTINATION share/${PROJECT_NAME}/cmake
)

# custom commands
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  ${CMAKE_CURRENT_LIST_DIR}/cmake/ament_cmake_lua_lib.cmake
)

ament_package()
