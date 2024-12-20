// Copyright 2025 Stanislav Mikhel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSIDL_LUACOMMON__DEFINITION_H_
#define ROSIDL_LUACOMMON__DEFINITION_H_

#include <lua.h>
#include <lauxlib.h>

/**
 * Type of the LUA wrapper for ROS message.
 * Negative value is used to object reference, positive - array size.
 */
typedef enum {
  /* Must be deleted with 'fini'. */
  /** Main pointer to the object. */
  IDL_LUA_OBJECT = -2,

  /* Don't need to call 'fini'. */
  /** Copy of the pointer to the object. */
  IDL_LUA_PTR = -1,
  /** Dynamic size array. */
  IDL_LUA_SEQ = 0
  /* Positive values for array size. */
} IDL_LUA_TYPE;

/** Wrapper for ROS message. */
typedef struct {
  /** Pointer to message or list of messages. */
  void *obj;
  /** Define pointer type. In the case of array define its length. */
  int value;
} idl_lua_msg_t;

/** 
 * Call 'require' to get library.
 *
 * \param lib library name.
 * \return sequence of lua API calls
 */
#define ROSIDL_LUA_REQUIRE(lib) \
  lua_getglobal(L, "require"); \
  lua_pushliteral(L, lib); \
  lua_call(L, 1, 2); \
  lua_pop(L, 3);

#endif  // ROSIDL_LUACOMMON__DEFINITION_H_
