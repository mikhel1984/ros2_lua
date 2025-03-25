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

#ifndef ROSIDL_LUACOMMON__UTILITY_H_
#define ROSIDL_LUACOMMON__UTILITY_H_

#include <lua.h>

#include "rosidl_luacommon/definition.h"

/** Prototype for data reallocation. */
typedef bool (*msg_mem_realloc)(idl_lua_msg_t *, size_t, bool);

/**
 * Find length of array or sequence, push result to stack.
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
int rosidl_luacommon_push_length(lua_State * L);

/**
 * Push simplified string representation of a message to the stack.
 *
 * \param[inout] L Lua stack.
 * \param[in] prefix Message name.
 * \return number of outputs.
 */
int rosidl_luacommon_push_msg_string(lua_State * L, const char * prefix);

/**
 * Update sequence length. Push boolean result of operation to the stack.
 *
 * \param[inout] L Lua stack.
 * \param[in] fn Message-related method for memory allocation.
 * \return number of outputs.
 */
int rosidl_luacommon_push_realloc(lua_State * L, msg_mem_realloc fn);

/**
 * Get pointer to message array if the index is not out of range.
 *
 * \param msg Lua message pointer.
 * \param ind Element index.
 * \return pointer to array or NULL.
 */
void * rosidl_luacommon_array_check_ind(idl_lua_msg_t * msg, int ind);

/**
 * Execute function from the given table.
 *
 * \param[inout] L Lua stack.
 * \param[in] table Name of the table with functions.
 * \param[in] top Expected stack top before function call.
 */
void rosidl_luacommon_field_apply(lua_State * L, const char * table, int top);

/**
 * Read elements from dictionary and set the message fields.
 * The dictionary is passed through the Lua stack.
 *
 * \param[inout] L Lua stack.
 * \return true in case of success.
 */
bool rosidl_luacommon_fill_from_table(lua_State * L);

/**
 * Check if the geven arguments are of different type, push false in this case.
 *
 * \param[inout] L Lua stack.
 * \return true when the arguments are different.
 */
bool rosidl_luacommon_push_wrong_args(lua_State * L);

/**
 * When the argement contains array or sequence, find its length, capacity and data pointer.
 *
 * \param[in] msg Lua message object pointer.
 * \param[out] size List size, zero for non-list.
 * \param[out] capacity List capacity, equal to size for array.
 * \return List data pointer or NULL.
 */
void * rosidl_luacommon_list_info(const idl_lua_msg_t * msg, size_t * size, size_t * capacity);

/**
 * Get list of keys or element type, push result to stack.
 *
 * \param[inout] L Lua stack.
 * \param[in] table Table name with fields.
 * \return number of outputs.
 */
int rosidl_luacommon_push_msg_keys(lua_State * L, const char * table);

/**
 * Push new message, fill it from C structure.
 *
 * \param[inout] L Lua stack.
 * \param[in] pos Stack position of the interface table.
 * \param[in] data Source C structure.
 */
void rosidl_luacommon_struct_to_msg(lua_State * L, int pos, void * data);

#endif  // ROSIDL_LUACOMMON__UTILITY_H_
