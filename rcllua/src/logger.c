
#include <lauxlib.h>

#include <rcutils/logging.h>
//#include <rcutils/allocator.h>

#include "utils.h"


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

static const rcl_lua_enum enum_log_types[] = {
  {"UNSET", RCUTILS_LOG_SEVERITY_UNSET},
  {"DEBUG", RCUTILS_LOG_SEVERITY_DEBUG},
  {"INFO", RCUTILS_LOG_SEVERITY_INFO},
  {"WARN", RCUTILS_LOG_SEVERITY_WARN},
  {"ERROR", RCUTILS_LOG_SEVERITY_ERROR},
  {"FATAL", RCUTILS_LOG_SEVERITY_FATAL},
  {NULL, -1}
};

void rcl_lua_add_logger_methods (lua_State* L)
{
  /* log message */
  lua_pushcfunction(L, rcl_lua_logger_log);
  lua_setfield(L, -2, "write_log");

  /* simple log */
  lua_pushcfunction(L, rcl_lua_logger_log_simp);
  lua_setfield(L, -2, "simp_log");

  /* log levels */
  rcl_lua_utils_add_enum(L, "LogLevel", enum_log_types);
}
