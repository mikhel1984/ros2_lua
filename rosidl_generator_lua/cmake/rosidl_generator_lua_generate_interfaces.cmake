# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
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

find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

set(Python3_FIND_UNVERSIONED_NAMES FIRST)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

find_package(rosidl_luacommon REQUIRED)

# Get a list of typesupport implementations from valid rmw implementations.
rosidl_generator_lua_get_typesupports(_typesupport_impls)

if(_typesupport_impls STREQUAL "")
  message(WARNING "No valid typesupport for Lua generator. Lua messages will not be generated.")
  return()
endif()

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_lua/${PROJECT_NAME}")
set(_generated_c_files "")


#foreach(_typesupport_impl ${_typesupport_impls})
#  set(_generated_extension_${_typesupport_impl}_files "")
#endforeach()

# Collect files for each interface type
set(_msg_list "")
set(_srv_list "")
set(_action_list "")

foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_idl_name}" _module_name)
  set(_src_c "${_output_path}/${_parent_folder}/${_module_name}.c")
  list(APPEND _generated_c_files ${_src_c})
  
  # separate msg / srv / action
  if(${_parent_folder} STREQUAL "msg")
    list(APPEND _msg_list ${_src_c})  
  elseif(${_parent_folder} STREQUAL "srv")
    list(APPEND _srv_list ${_src_c})
  else()
    list(APPEND _action_list ${_src_c})
  endif()    
endforeach()

# Add lua binding
if(NOT _msg_list STREQUAL "")
  list(INSERT _msg_list 0 "${_output_path}/msg/msg_lib.c")
  list(APPEND _generated_c_files "${_output_path}/msg/msg_lib.c")
endif()
if(NOT _srv_list STREQUAL "")
  list(INSERT _srv_list 0 "${_output_path}/srv/srv_lib.c")
  list(APPEND _generated_c_files "${_output_path}/srv/srv_lib.c")
endif()
if(NOT _action_list STREQUAL "")
  list(INSERT _action_list 0 "${_output_path}/action/action_lib.c")
  list(APPEND _generated_c_files "${_output_path}/action/action_lib.c")
endif()

file(MAKE_DIRECTORY "${_output_path}")

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_lua_BIN}"
  ${rosidl_generator_lua_GENERATOR_FILES}
  "${rosidl_generator_lua_TEMPLATE_DIR}/idl.c.em"
  "${rosidl_generator_lua_TEMPLATE_DIR}/msg.c.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_lua__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_lua_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

set(_target_suffix "__lua")

set_property(
  SOURCE ${_generated_c_files}
  PROPERTY GENERATED 1
)

# Set generation rule
add_custom_command(
  OUTPUT ${_generated_c_files}
  COMMAND Python3::Interpreter
  ARGS ${rosidl_generator_lua_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  --typesupport-impls "${_typesupport_impls}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating Lua code for ROS interfaces"
  VERBATIM
)

if(TARGET ${rosidl_generate_interfaces_TARGET}${_target_suffix})
  message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}${_target_suffix} already exists")
else()
  add_custom_target(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    DEPENDS
    ${_generated_c_files})
endif()

# Export target so downstream interface packages can link to it
set(rosidl_generator_lua_suffix "__rosidl_generator_lua")

set(_target_name_lib "${rosidl_generate_interfaces_TARGET}${rosidl_generator_lua_suffix}")

add_library(${_target_name_lib} SHARED ${_generated_c_files})
target_link_libraries(${_target_name_lib}
  ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c)
add_dependencies(
  ${_target_name_lib}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__rosidl_typesupport_c
)

# Additional definitions
set(_luacommon_dir "${rosidl_luacommon_DIR}/../../../include")
normalize_path(_luacommon_dir "${_luacommon_dir}")

target_include_directories(${_target_name_lib}
  PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
  ${_luacommon_dir}
)

rosidl_get_typesupport_target(c_typesupport_target "${rosidl_generate_interfaces_TARGET}" "rosidl_typesupport_c")

# Compile lua librariesy
if(NOT _msg_list STREQUAL "")
  add_library(msg SHARED ${_msg_list})
  set_target_properties(msg PROPERTIES 
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${_output_path}
  )
  target_link_libraries(msg
    #${rosidl_generate_interfaces_TARGET}__rosidl_generator_c
    ${c_typesupport_target}
    )
#  add_dependencies(
#  msg
#  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
#  ${rosidl_generate_interfaces_TARGET}__rosidl_typesupport_c
#)
  #target_include_directories(msg PRIVATE
    #${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
    #${_luacommon_dir}
    #)
  ament_target_dependencies(msg
    "rosidl_runtime_c"
    #"rosidl_typesupport_c"
    #"rosidl_typesupport_interface"
    "rosidl_luacommon"
  )
endif()
if(NOT _srv_list STREQUAL "")
  add_library(srv SHARED ${_srv_list})
  set_target_properties(srv PROPERTIES 
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${_output_path}
  )
  target_link_libraries(srv
    ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c)
  target_include_directories(srv PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
    ${_luacommon_dir})
endif()
if(NOT _action_list STREQUAL "")
  add_library(action SHARED ${_action_list})
  set_target_properties(action PROPERTIES 
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${_output_path}
  )
  target_link_libraries(action
    ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c)
  target_include_directories(action PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
    ${_luacommon_dir})
endif()

# Install
if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  find_package(rcllua_cmake REQUIRED)
  if(NOT _msg_list STREQUAL "")
    rcllua_cmake_install_clib(${PROJECT_NAME} msg)
  endif()
  if(NOT _srv_list STREQUAL "")
    rcllua_cmake_install_clib(${PROJECT_NAME} srv)  
  endif()
  if(NOT _action_list STREQUAL "")
    rcllua_cmake_install_clib(${PROJECT_NAME} action)  
  endif()
endif()


