cmake_minimum_required(VERSION 3.8)
project(rcllua_examples)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcllua REQUIRED)
find_package(rcllua_cmake REQUIRED)

# rcllua examples
rcllua_cmake_executable(${PROJECT_NAME}/publisher.lua
  simple_publisher)
rcllua_cmake_executable(${PROJECT_NAME}/subscription.lua
  simple_subscription)
rcllua_cmake_executable(${PROJECT_NAME}/service.lua
  simple_service)
rcllua_cmake_executable(${PROJECT_NAME}/client.lua
  simple_client)

# rclbind examples
rcllua_cmake_executable(raw/timer.lua raw_timer)

# if need
# copy files to install/lib directory, add to Lua path
rcllua_cmake_install_lib(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # apply tests
  find_package(rcllua_unit REQUIRED)
  rcllua_unit_tests(
    tests/test_lib.lua
  )
endif()

ament_package()
