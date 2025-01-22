# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
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

macro(rcllua_generate_interfaces interface_name)
  
  # find path to 'share' message directory
  find_package(${interface_name} REQUIRED)
  set(_cm_dir ${${interface_name}_DIR})
  get_filename_component(_src_path "${${interface_name}_DIR}/.." ABSOLUTE)
  
  set(_idl_tuples "")
  set(_non_idl_tuples "")
  set(_interface_tuples "")
  
  # simply collect idl and not idl files
  if(EXISTS "${_src_path}/msg")
    file(GLOB _idl_files RELATIVE "${_src_path}" "${_src_path}/msg/*.idl")
    foreach(_file ${_idl_files})
      list(APPEND _idl_tuples "${_src_path}:${_file}")
      list(APPEND _interface_tuples "${_src_path}:${_file}")
    endforeach()
    file(GLOB _msg_files RELATIVE "${_src_path}" "${_src_path}/msg/*.msg")
    foreach(_file ${_msg_files})
      list(APPEND _non_idl_tuples "${_src_path}:${_file}")
      list(APPEND _interface_tuples "${_src_path}:${_file}")
    endforeach()    
  endif()
  
  if(EXISTS "${_src_path}/srv")
    file(GLOB _idl_files RELATIVE "${_src_path}" "${_src_path}/srv/*.idl")
    foreach(_file ${_idl_files})
      list(APPEND _idl_tuples "${_src_path}:${_file}")
      list(APPEND _interface_tuples "${_src_path}:${_file}")
    endforeach()
    file(GLOB _msg_files RELATIVE "${_src_path}" "${_src_path}/srv/*.msg")
    foreach(_file ${_msg_files})
      list(APPEND _non_idl_tuples "${_src_path}:${_file}")
      list(APPEND _interface_tuples "${_src_path}:${_file}")
    endforeach()    
  endif()
 
  # Check for any action or service interfaces
  # Which have implicit dependencies that need to be found
  foreach(_tuple ${_interface_tuples})
    string(REGEX REPLACE ".*:([^:]*)$" "\\1" _tuple_file "${_tuple}")
    get_filename_component(_parent_dir "${_tuple_file}" DIRECTORY)
    get_filename_component(_parent_dir ${_parent_dir} NAME)

    if("${_parent_dir}" STREQUAL "action")
      # Actions depend on the packages service_msgs and action_msgs
      find_package(service_msgs QUIET)
      if(NOT ${service_msgs_FOUND})
        message(FATAL_ERROR
          "Unable to generate action interface for '${_tuple_file}'. "
          "In order to generate action interfaces you must add a depend tag "
          "for 'service_msgs' in your package.xml.")
      endif()
      ament_export_dependencies(service_msgs)
      list_append_unique(_ARG_DEPENDENCIES "service_msgs")
      find_package(action_msgs QUIET)
      if(NOT ${action_msgs_FOUND})
        message(FATAL_ERROR
          "Unable to generate action interface for '${_tuple_file}'. "
          "In order to generate action interfaces you must add a depend tag "
          "for 'action_msgs' in your package.xml.")
      endif()
      ament_export_dependencies(action_msgs)
      list_append_unique(_ARG_DEPENDENCIES "action_msgs")

      # It is safe to break out of the loop since services only depend on service_msgs
      # Which has already been found above
      break()
    elseif("${_parent_dir}" STREQUAL "srv")
      # Services depend on service_msgs
      find_package(service_msgs QUIET)
      if(NOT ${service_msgs_FOUND})
        message(FATAL_ERROR
          "Unable to generate service interface for '${_tuple_file}'. "
          "In order to generate service interfaces you must add a depend tag "
          "for 'service_msgs' in your package.xml.")
      endif()
      ament_export_dependencies(service_msgs)
      list_append_unique(_ARG_DEPENDENCIES "service_msgs")
    endif()
  endforeach()
  
  # collect all interface files from dependencies
  set(_dep_files)
  foreach(_dep ${ARGN})
    if(NOT ${_dep}_FOUND)
      message(FATAL_ERROR "rosidl_generate_interfaces() the passed dependency "
        "'${_dep}' has not been found before using find_package()")
    endif()
    foreach(_idl_file ${${_dep}_IDL_FILES})
      rosidl_find_package_idl(_abs_idl_file "${_dep}" "${_idl_file}")
      list(APPEND _dep_files "${_abs_idl_file}")
    endforeach()
  endforeach()

  # collect package names of recursive dependencies which contain interface files
  set(_recursive_dependencies)
  foreach(_dep ${ARGN})
    if(DEFINED ${_dep}_IDL_FILES)
      list_append_unique(_recursive_dependencies "${_dep}")
    endif()
    foreach(_dep2 ${${_dep}_RECURSIVE_DEPENDENCIES})
      if(DEFINED ${_dep2}_IDL_FILES)
        list_append_unique(_recursive_dependencies "${_dep2}")
      endif()
    endforeach()
  endforeach()
  
  set(_non_idl_files "")
  foreach(_tuple ${_non_idl_tuples})
    string(REGEX REPLACE ":([^:]*)$" "/\\1" _non_idl_file "${_tuple}")
    list(APPEND _non_idl_files "${_non_idl_file}")
  endforeach()

  add_custom_target(
    ${interface_name} ALL
    DEPENDS
    ${_non_idl_files}
    ${_dep_files}
    SOURCES
    ${_non_idl_files}
  )

  # generators must be executed in topological order
  # which is ensured by every generator finding its dependencies first
  # and then registering itself as an extension
  set(rosidl_generate_interfaces_TARGET ${interface_name})
  set(rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES ${_recursive_dependencies})  
  set(rosidl_generate_interfaces_IDL_TUPLES ${_idl_tuples})

  set(rosidl_generate_interfaces_ABS_IDL_FILES)
  foreach(_idl_tuple ${rosidl_generate_interfaces_IDL_TUPLES})
    string(REGEX REPLACE ":([^:]*)$" "/\\1" _abs_idl_file "${_idl_tuple}")
    list(APPEND rosidl_generate_interfaces_ABS_IDL_FILES "${_abs_idl_file}")
  endforeach()
  
  rosidl_lua_generate_lib(${interface_name}) 

endmacro()
