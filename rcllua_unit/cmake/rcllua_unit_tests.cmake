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

include(CTest)

# Executable name
set(RCLLUA_CALL lua)

# Call each file test
function (rcllua_unit_tests)
  # call test for each input lua file
  foreach(file ${ARGV})
    add_test(NAME ${file} COMMAND ${RCLLUA_CALL} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
  endforeach()
endfunction()
