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

#include <lauxlib.h>

#include <rcutils/logging.h>

#include "utils.h"

/**
 * Write message full form into log.
 *
 * Arguments:
 * - severity (enum)
 * - node name
 * - message text
 * - function name
 * - file name
 * - line number (int)
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_logger_log (lua_State* L)
{
  /* arg1 - severity */
  int severity = luaL_checkinteger(L, 1);
  /* arg2 - name */
  const char* name = luaL_checkstring(L, 2);
  /* arg3 - message */
  const char* message = luaL_checkstring(L, 3);
  /* arg4 - function name */
  const char* func_name = luaL_checkstring(L, 4);
  /* arg5 - file name */
  const char* file_name = luaL_checkstring(L, 5);
  /* arg6 - line number */
  lua_Integer num = luaL_checkinteger(L, 6);
  luaL_argcheck(L, num > 0, 6, "negative line number");

  /* write */
  RCUTILS_LOGGING_AUTOINIT;
  rcutils_log_location_t logging_location = {func_name, file_name, (uint64_t) num};
  rcutils_log(&logging_location, severity, name, "%s", message);

  return 0;
}

/**
 * Write message short form into log.
 *
 * Arguments:
 * - severity (enum)
 * - node name
 * - message text
 *
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int rcl_lua_logger_log_simp (lua_State* L)
{
  /* arg1 - severity */
  int severity = luaL_checkinteger(L, 1);
  /* arg2 - name */
  const char* name = luaL_checkstring(L, 2);
  /* arg3 - message */
  const char* message = luaL_checkstring(L, 3);

  /* write */
  RCUTILS_LOGGING_AUTOINIT;
  rcutils_log(NULL, severity, name, "%s", message);

  return 0;
}

/** List of severity values */
static const rcl_lua_enum enum_log_types[] = {
  {"UNSET", RCUTILS_LOG_SEVERITY_UNSET},
  {"DEBUG", RCUTILS_LOG_SEVERITY_DEBUG},
  {"INFO", RCUTILS_LOG_SEVERITY_INFO},
  {"WARN", RCUTILS_LOG_SEVERITY_WARN},
  {"ERROR", RCUTILS_LOG_SEVERITY_ERROR},
  {"FATAL", RCUTILS_LOG_SEVERITY_FATAL},
  {NULL, -1}
};

/* Add logging to library */
void rcl_lua_add_logger_methods (lua_State* L)
{
  /* full log message */
  lua_pushcfunction(L, rcl_lua_logger_log);       // push function
  lua_setfield(L, -2, "write_log");               // pop, lib['write_log'] = function

  /* simple log message */
  lua_pushcfunction(L, rcl_lua_logger_log_simp);  // push function
  lua_setfield(L, -2, "simp_log");                // pop, lib['simp_log'] = function

  /* log levels */
  rcl_lua_utils_add_enum(L, "LogLevel", enum_log_types);  // lib['LogLevel'] = {...}
}
