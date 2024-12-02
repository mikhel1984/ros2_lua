
include(CTest)

# Executable name
set(LUA_EXEC lua)

# Call each file test
function (rcllua_unit_tests)  
  # call test for each input lua file
  foreach(file ${ARGV})
    add_test(NAME ${file} COMMAND ${LUA_EXEC} ${CMAKE_CURRENT_SOURCE_DIR}/${file})  
  endforeach()
endfunction()

