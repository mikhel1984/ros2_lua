
# Copy lua files, add environment hook
# src_name - name of the directory with scripts
macro(ament_install_lua_lib src_name)
  install(
    DIRECTORY ${src_name}
    DESTINATION lib
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
macro(ament_install_lua_clib src_name)
  install(
    TARGETS ${src_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  if(NOT DEFINED _AMENT_CMAKE_LUAC_ENVIRONMENT_HOOK_REGISTERED)
    set(_AMENT_CMAKE_LUAC_ENVIRONMENT_HOOK_REGISTERED TRUE)
    
    find_package(ament_cmake_core QUIET REQUIRED)
    
    ament_environment_hooks(
      "${rcllua_cmake_DIR}/templates/env_hook_lua_c.sh.in")
  endif()  
endmacro()
