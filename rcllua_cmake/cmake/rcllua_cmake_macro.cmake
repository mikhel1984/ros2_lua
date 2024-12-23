# Copyright 2025 Stanislav Mikhel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copy lua files, add environment hook
# src_name - name of the directory with scripts
macro(rcllua_cmake_install_lib src_name)
  install(
    DIRECTORY ${src_name}
    DESTINATION lib/lua
  )

  if(NOT DEFINED _AMENT_CMAKE_LUA_ENVIRONMENT_HOOK_REGISTERED)
    set(_AMENT_CMAKE_LUA_ENVIRONMENT_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core QUIET REQUIRED)

    ament_environment_hooks(
      "${rcllua_cmake_DIR}/templates/env_hook_lua.sh.in")
  endif()
endmacro()

# Copy dynamic library, add environment hook
# src_name - name of the dynamic library file
macro(rcllua_cmake_install_clib proj_name src_name)
  install(
    TARGETS ${src_name}
    ARCHIVE DESTINATION lib/lua/${proj_name}
    LIBRARY DESTINATION lib/lua/${proj_name}
    RUNTIME DESTINATION bin/lua/${proj_name}
  )

  if(NOT DEFINED _AMENT_CMAKE_LUAC_ENVIRONMENT_HOOK_REGISTERED)
    set(_AMENT_CMAKE_LUAC_ENVIRONMENT_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core QUIET REQUIRED)

    ament_environment_hooks(
      "${rcllua_cmake_DIR}/templates/env_hook_lua_c.sh.in")
  endif()
endmacro()

# Lua binary file
set(LUA_EXEC /usr/local/bin/lua)

# Make executable file to call with 'ros2 run'
# src_name - source file name in package directory
# exec_name - desired executable name
macro(rcllua_cmake_executable src_name exec_name)
  set(_out_dir ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
  file(MAKE_DIRECTORY "${_out_dir}")
  # file content
  file(WRITE "${_out_dir}/${exec_name}"
    "#!${LUA_EXEC}\n"
    "local script = loadfile('${CMAKE_SOURCE_DIR}/${src_name}')\n"
    "script()"
  )
  file(CHMOD "${_out_dir}/${exec_name}"
    PERMISSIONS
      OWNER_READ OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
endmacro()
