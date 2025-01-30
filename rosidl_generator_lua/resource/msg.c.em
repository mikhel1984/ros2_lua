// Created from rosidl_generator_lua/resource/msg.c
// Generated code does not contain a copyright notice
@{
from rosidl_generator_lua import NUMERIC_LUA_TYPES, sequence_metatable, make_prefix
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME
from rosidl_parser.definition import NamespacedType

include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)

header_files = [
    'limits.h',
    'float.h',
    'stdint.h',
    'stdbool.h',
    'rosidl_runtime_c/visibility_control.h',
    include_base + '__struct.h',
    include_base + '__functions.h',
    include_base + '__type_support.h',
    'rosidl_luacommon/definition.h',
    'rosidl_luacommon/utility.h']
}@

@[for header_file in header_files]@
@{
repeated_header_file = header_file in include_directives
}@
@[    if repeated_header_file]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
@[    if '/' not in header_file]@
#include <@(header_file)>
@[    else]@
#include "@(header_file)"
@[    end if]@
@[end for]@

@{
have_not_included_primitive_arrays = True
have_not_included_string = True
have_not_included_wstring = True
nested_types = set()
}@
@[for member in message.structure.members]@
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
header_files = []
if isinstance(member.type, AbstractNestedType) and have_not_included_primitive_arrays:
    have_not_included_primitive_arrays = False
    header_files += [
        'rosidl_runtime_c/primitives_sequence.h',
        'rosidl_runtime_c/primitives_sequence_functions.h']
if isinstance(type_, AbstractString) and have_not_included_string:
    have_not_included_string = False
    header_files += [
        'rosidl_runtime_c/string.h',
        'rosidl_runtime_c/string_functions.h']
#if isinstance(type_, AbstractWString) and have_not_included_wstring:
#    have_not_included_wstring = False
#    header_files += [
#        'rosidl_runtime_c/u16string.h',
#        'rosidl_runtime_c/u16string_functions.h']
}@
@[if header_files]@
@[  for header_file in header_files]@
@[    if header_file in include_directives]@
// already included above
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"
@[  end for]@
@[end if]@

@{
if isinstance(member.type, AbstractNestedType) and isinstance(member.type.value_type, NamespacedType):
    nested_types.add((*member.type.value_type.namespaces, member.type.value_type.name))
}@
@[end for]@
@[if nested_types]@
// Nested array functions includes
@[  for type_ in sorted(nested_types)]@
@{
nested_header = '/'.join(type_[:-1] + ('detail', convert_camel_case_to_lower_case_underscore(type_[-1]),))
nested_header += '__functions.h'
}@
@[    if nested_header in include_directives]@
// already included above
@[    else]@
@{include_directives.add(nested_header)}@
@[    end if]@
#include "@(nested_header)"
@[  end for]@
// end nested array functions include
@[end if]@

@{
msg_components = message.structure.namespaced_type.namespaced_name()
msg_typename = '__'.join(msg_components)
msg_prefix = make_prefix(message)
msg_getters = []
msg_setters = []
msg_metatable = msg_typename + '__mt'
}@
static int @(msg_prefix)__lcall (lua_State* L);

/**
 * Message constructor.
 * in: nil or other message or table
 * out: new message
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__lnew (lua_State* L) {
  /* message object */
  @(msg_typename)* msg = @(msg_typename)__create();
  if (NULL == msg) {
    luaL_error(L, "failed to create message");
  }
  if (!@(msg_typename)__init(msg)) {
    @(msg_typename)__destroy(msg);
    luaL_error(L, "failed to init message");
  }

  /* object wrapper */
  idl_lua_msg_t* ptr = lua_newuserdata(L, sizeof(idl_lua_msg_t));  // push object
  ptr->obj = msg;                // message
  ptr->value = IDL_LUA_OBJECT;   // pointer type

  /* add metamethods */
  luaL_getmetatable(L, "@(msg_metatable)");   // push metatable
  lua_setmetatable(L, -2);       // pop metatable

  /* table based initialization */
  if (LUA_TTABLE == lua_type(L, 2)) {
    lua_replace(L, 1);         // pop, move userdata to first place
    rosidl_luacommon_fill_from_table(L);
    lua_settop(L, 1);          // pop, remove call result
  }

  return 1;
}

/**
 * Message destruction.
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__lgc (lua_State* L) {
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (IDL_LUA_OBJECT == ptr->value && NULL != ptr->obj) {
    @(msg_typename)__fini(ptr->obj);
    @(msg_typename)__destroy(ptr->obj);
    ptr->obj = NULL;
  }

  return 0;
}

/**
 * Check equality of two messages.
 * in: (msg1, msg2)
 * out: bool
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__leq (lua_State* L) {
  if (rosidl_luacommon_push_wrong_args(L)) {
    return 1;
  }

  /* compare data */
  idl_lua_msg_t* p1 = lua_touserdata(L, 1);
  idl_lua_msg_t* p2 = lua_touserdata(L, 2);

  if (p1->value < IDL_LUA_SEQ && p2->value < IDL_LUA_SEQ) {
    /* object or reference */
    lua_pushboolean(L, @(msg_typename)__are_equal(p1->obj, p2->obj));
  } else if (p1->value >= IDL_LUA_SEQ && p2->value >= IDL_LUA_SEQ) {
    /* lists */
    @(msg_typename)__Sequence s1, s2;
    if (p1->value == IDL_LUA_SEQ) {
      s1 = *(@(msg_typename)__Sequence*) p1->obj;
    } else {
      /* array, to Sequence object */
      s1.data = p1->obj;
      s1.size = s1.capacity = p1->value;
    }
    if (p2->value == IDL_LUA_SEQ) {
      s2 = *(@(msg_typename)__Sequence*) p2->obj;
    } else {
      /* array, to Sequence object */
      s2.data = p2->obj;
      s2.size = s2.capacity = p2->value;
    }
    lua_pushboolean(L, @(msg_typename)__Sequence__are_equal(&s1, &s2));
  } else {
    lua_pushboolean(L, false);
  }

  return 1;
}

/**
 * Copy data.
 * in: other_msg
 * out: bool
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__lcopy (lua_State* L) {
  if (rosidl_luacommon_push_wrong_args(L)) {
    return 1;
  }

  /* data */
  idl_lua_msg_t* dst = lua_touserdata(L, 1);
  idl_lua_msg_t* src = lua_touserdata(L, 2);
  bool done = false;

  if (dst->value < IDL_LUA_SEQ && src->value < IDL_LUA_SEQ) {
    /* objects */
    done = @(msg_typename)__copy(src->obj, dst->obj);
  } else if (IDL_LUA_SEQ == dst->value) {
    /* list */
    if (IDL_LUA_SEQ == src->value) {
      done = @(msg_typename)__Sequence__copy(src->obj, dst->obj);
    } else if (src->value > IDL_LUA_SEQ) {
      /* from array */
      @(msg_typename)__Sequence tmp;
      tmp.data = src->obj;
      tmp.size = tmp.capacity = (size_t) src->value;
      done = @(msg_typename)__Sequence__copy(&tmp, dst->obj);
    }
  } else if (dst->value > IDL_LUA_SEQ) {
    /* array */
    @(msg_typename) *a = dst->obj, *b = NULL;
    if (src->value > IDL_LUA_SEQ && dst->value == src->value) {
      b = src->obj;
    } else if (src->value == IDL_LUA_SEQ) {
      /* from list */
      @(msg_typename)__Sequence* seq = src->obj;
      if (seq->size == (size_t) dst->value) {
        b = seq->data;
      }
    }
    if (b) {
      done = true;
      for (int i = 0; i < dst->value; i++) {
        done = done && @(msg_typename)__copy(b++, a++);
      }
    }
  }

  lua_pushboolean(L, done);
  return 1;
}

/**
 * Array length.
 * out: length or nil
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__llen (lua_State* L)
{
  return rosidl_luacommon_push_length(L);
}

/**
 * String representation.
 * out: string
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__lstr (lua_State* L)
{
  return rosidl_luacommon_push_msg_string(L, "@(msg_typename)");
}

/**
 * Get object fields or array type.
 * out: "static"/"dynamic" for array or {"field1", "field2", ..} for message.
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__lbnot (lua_State* L)
{
  return rosidl_luacommon_push_msg_keys(L, "getters");
}

/**
 * Reallocate memory. Copy old data if need.
 * It is assumed that new size is greater then the previous one.
 * \param[inout] ptr Pointer to Lua message structure.
 * \param[in] n New size.
 * \param[in] copy Flag to make copy of the stored data.
 * \return true in case of success.
 */
bool @(msg_prefix)__do_resize (idl_lua_msg_t* ptr, size_t n, bool copy)
{
  @(msg_typename)__Sequence* seq = ptr->obj;
  bool done = true;

  if (copy) {
    @(msg_typename)__Sequence newseq;
    if (@(msg_typename)__Sequence__init(&newseq, n) &&
        @(msg_typename)__Sequence__copy(seq, &newseq))
    {
      /* swap */
      @(msg_typename)__Sequence tmp = *seq;
      *seq = newseq;
      seq->size = n;   // size become reduced after copy
      /* remove old */
      @(msg_typename)__Sequence__fini(&tmp);
    } else {
      done = false;
    }
  } else {
    if (seq->capacity) {
      /* free old memory */
      @(msg_typename)__Sequence__fini(seq);
    }
    done = @(msg_typename)__Sequence__init(seq, n);
  }
  return done;
}

/**
 * Change sequence size.
 * in: new_size
 * out: bool
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__lresize (lua_State* L)
{
  return rosidl_luacommon_push_realloc(L, @(msg_prefix)__do_resize);
}

@# setters
@[for member in message.structure.members]@
@[  if len(message.structure.members) == 1 and member.name == EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME]@
@[    continue]@
@[  end if]@
@{
setter_ = '_'.join((msg_prefix, '_set', member.name))
}@

/**
 * Set '@(member.name)'.
 * in: new_value
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(setter_) (lua_State* L) {
  /* stack [object, field name, new value] */
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  @(msg_typename)* ros_msg = ptr->obj;
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
}@
@[  if isinstance(type_, NamespacedType)]@
@{
nested_type = '__'.join(type_.namespaced_name())
nested_metatable = nested_type + '__mt'
}@

  /* nested type */
  luaL_getmetatable(L, "@(nested_metatable)");         // push metatable
  if (lua_isnil(L, -1)) {
    luaL_error(L, "@(nested_type) not found");
  }
  if (lua_getfield(L, -1, "copy") != LUA_TFUNCTION) {  // push function
    luaL_error(L, "no method for object copy");
  }

  /* wrap object to call metamethod */
  idl_lua_msg_t* dst = lua_newuserdata(L, sizeof(idl_lua_msg_t));  // push object
@[    if isinstance(member.type, AbstractNestedType)]@
@[      if isinstance(member.type, AbstractSequence)]@
  dst->obj = &(ros_msg->@(member.name));      // pointer to sequence
  dst->value = IDL_LUA_SEQ;
@[      else]@
  dst->obj = ros_msg->@(member.name);         // array
  dst->value = @(member.type.size);
@[      end if]@
@[    else]@
  dst->obj = &(ros_msg->@(member.name));      // pointer to object
  dst->value = IDL_LUA_PTR;
@[    end if]@
  lua_pushvalue(L, -3);                       // push metatable (duplicate)
  lua_setmetatable(L, -2);                    // pop metatable

  /* stack [..., function, dst] */
  lua_pushvalue(L, 3);                        // push argument (duplicate)
  lua_call(L, 2, 1);                          // call copy(L-2, L-1), 2 inputs 1 result
@[  elif isinstance(member.type, AbstractNestedType)]@

  /* primitive type sequence */
  luaL_getmetatable(L, "@(sequence_metatable(member.type.value_type))");  // push mt
  if (lua_isnil(L, -1)) {
    luaL_error(L, "@(sequence_metatable(member.type.value_type)) not found");
  }
  if (lua_getfield(L, -1, "copy") != LUA_TFUNCTION) {   // push function
    luaL_error(L, "no method for object copy");
  }

  /* create object to call metamethod */
  idl_lua_msg_t* dst = lua_newuserdata(L, sizeof(idl_lua_msg_t));   // push object
@[    if isinstance(member.type, AbstractSequence)]@
  dst->obj = &(ros_msg->@(member.name));      // sequence
  dst->value = IDL_LUA_SEQ;
@[    else]@
  dst->obj = ros_msg->@(member.name);         // array
  dst->value = @(member.type.size);
@[    end if]@
  lua_pushvalue(L, -3);                       // push metatable (duplicate)
  lua_setmetatable(L, -2);                    // pop metatable

  /* stack [..., function, dst] */
  lua_pushvalue(L, 3);                        // push argument (duplicate)
  lua_call(L, 2, 1);                          // copy(L-2, L-1)
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'char']@

  const char* value = luaL_checkstring(L, 3);
  ros_msg->@(member.name) = value[0];
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'boolean']@

  if (LUA_TBOOLEAN != lua_type(L, 3)) {
    luaL_error(L, "expected boolean");
  }
  ros_msg->@(member.name) = lua_toboolean(L, 3);
@[  elif isinstance(member.type, BasicType) and member.type.typename in NUMERIC_LUA_TYPES]@
@{
type_dict = NUMERIC_LUA_TYPES[member.type.typename]
}@

  @(type_dict['var']) value = @(type_dict['fn'])(L, 3);
@#  check for unsigned value
@[    if member.type.typename.startswith('u') ]@
  if (value < 0 || ((size_t) value) > @(type_dict['max'])) {
@[    else]@
  if (value < @(type_dict['min']) || value > @(type_dict['max'])) {
@[    end if]@
    luaL_error(L, "value out of range");
  }
  ros_msg->@(member.name) = value;
@[  elif isinstance(member.type, AbstractString)]@

  const char* value = luaL_checkstring(L, 3);
  rosidl_runtime_c__String__assign(&ros_msg->@(member.name), value);
@[  elif isinstance(member.type, AbstractWString)]@
@# ignore
    (void) ros_msg;
@[  else]@
@{
assert False, ("unknown type " + member.type.typename)
}@
@[  end if]@
  return 0;
}
@{
msg_setters.append((member.name, setter_))
}@
@[end for]@

@#  getters
@[for member in message.structure.members]@
@[  if len(message.structure.members) == 1 and member.name == EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME]@
@[    continue]@
@[  end if]@
@{
getter_ = '_'.join((msg_prefix, '_get', member.name))
}@

/**
 * Get '@(member.name)'.
 * out: current_value
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(getter_) (lua_State* L) {
  /* stack [object, field name] */
  idl_lua_msg_t* src = lua_touserdata(L, 1);
  @(msg_typename)* ros_msg = src->obj;
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
}@
@[  if isinstance(type_, NamespacedType)]@
@{
nested_type = '__'.join(type_.namespaced_name())
mtbl = nested_type + '__mt'
}@

  /* return new object */
  idl_lua_msg_t* ptr = lua_newuserdata(L, sizeof(idl_lua_msg_t));  // push object
@[    if isinstance(member.type, AbstractNestedType)]@
@[      if isinstance(member.type, AbstractSequence)]@
  ptr->obj = &(ros_msg->@(member.name));      // pointer to sequence
  ptr->value = IDL_LUA_SEQ;
@[      else]@
  ptr->obj = ros_msg->@(member.name);         // array
  ptr->value = @(member.type.size);
@[      end if]@
@[    else]@
  ptr->obj = &(ros_msg->@(member.name));      // pointer to object
  ptr->value = IDL_LUA_PTR;
@[    end if]@
  luaL_getmetatable(L, "@(mtbl)");            // push metatable
  lua_setmetatable(L, -2);                    // pop metatable
@[  elif isinstance(member.type, AbstractNestedType)]@

  idl_lua_msg_t* ptr = lua_newuserdata(L, sizeof(idl_lua_msg_t));  // push object
@[    if isinstance(member.type, AbstractSequence)]@
  ptr->obj = &(ros_msg->@(member.name));
  ptr->value = IDL_LUA_SEQ;
@[    else]@
  ptr->obj = ros_msg->@(member.name);
  ptr->value = @(member.type.size);
@[    end if]@
  luaL_getmetatable(L, "@(sequence_metatable(member.type.value_type))");  // push mt
  lua_setmetatable(L, -2);                    // pop metatable
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'char']@

  lua_pushinteger(L, ros_msg->@(member.name));
@[  elif isinstance(member.type, AbstractString)]@

  rosidl_runtime_c__String str = ros_msg->@(member.name);
  lua_pushlstring(L, str.data, str.size);
@[  elif isinstance(member.type, AbstractWString)]@
@# ignore
    (void) ros_msg;
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'boolean']@

  lua_pushboolean(L, ros_msg->@(member.name));
@[  elif isinstance(member.type, BasicType) and member.type.typename in NUMERIC_LUA_TYPES]@
@{
type_dict = NUMERIC_LUA_TYPES[member.type.typename]
}@

  @(type_dict['ifn'])(L, ros_msg->@(member.name));
@[  else]@
@{
assert False, ("unknown type " + member.type.typename)
}@
@[  end if]@
  return 1;
}
@{
msg_getters.append((member.name, getter_))
}@
@[end for]@

/**
 * Main getter function.
 * in: field_name
 * out: current_value
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__lindex (lua_State* L) {
  idl_lua_msg_t* msg = lua_touserdata(L, 1);

  if (msg->value >= IDL_LUA_SEQ) {
    lua_Integer n = luaL_checkinteger(L, 2);
    /* object list, same metatable, get by index */
    @(msg_typename)* lst = rosidl_luacommon_array_check_ind(msg, n);

    if (lst) {
      idl_lua_msg_t* res = lua_newuserdata(L, sizeof(idl_lua_msg_t));  // push obj
      res->obj = &(lst[n-1]);                   // index from 1
      res->value = IDL_LUA_PTR;
      lua_getmetatable(L, 1);                   // push metatable
      lua_setmetatable(L, -2);                  // pop metatable, copy to new object
    } else {
      lua_pushnil(L);
    }
  } else {
    rosidl_luacommon_field_apply(L, "getters", 2);
  }

  return 1;
}

/**
 * Main setter function.
 * in: field_name, new_value
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__lnewindex (lua_State* L) {
  idl_lua_msg_t* msg = lua_touserdata(L, 1);

  if (msg->value >= IDL_LUA_SEQ) {
    lua_Integer n = luaL_checkinteger(L, 2);
    /* object list, same metatable, by index */
    @(msg_typename)* lst = rosidl_luacommon_array_check_ind(msg, n);

    /* right part */
    idl_lua_msg_t* src = luaL_checkudata(L, 3, "@(msg_metatable)");
    if (src->value >= IDL_LUA_SEQ) {
      luaL_error(L, "different types");
    }
    if (lst) {
      @(msg_typename)__copy(src->obj, &(lst[n-1]));
    }
  } else {
    rosidl_luacommon_field_apply(L, "setters", 3);
  }

  return 0;
}

/**
 * Execute additional (copy, resize, init) operations using message 'call'.
 * in: another message or new size or initialization table
 * out: bool
 * \param[inout] L Lua stack.
 * \return number of outputs.
 */
static int @(msg_prefix)__lcall (lua_State* L) {
  bool done = false;
  int tp = lua_type(L, 2);

  if (LUA_TNONE == tp) {
    /* make new message */
    @(msg_prefix)__lnew(L);   // push new message
    lua_insert(L, 1);         // swap
    @(msg_prefix)__lcopy(L);  // push fill result
    if (lua_toboolean(L, -1)) {
      lua_pushvalue(L, 1);
    } else {
      lua_pushnil(L);
    }
    return 1;

  } else if (LUA_TUSERDATA == tp) {
    /* copy values */
    return @(msg_prefix)__lcopy(L);

  } else if (LUA_TNUMBER == tp) {
    /* resize object */
    return @(msg_prefix)__lresize(L);

  } else if (LUA_TTABLE == tp) {
    int len = luaL_len(L, 2);
    idl_lua_msg_t* msg = lua_touserdata(L, 1);

    if (len > 0 && msg->value >= IDL_LUA_SEQ) {
      /* element-wise copy */
      size_t arr_len = 0, arr_cap = 0;
      @(msg_typename)* lst = rosidl_luacommon_list_info(msg, &arr_len, &arr_cap);
      if (arr_len != (size_t) len) {
        if (IDL_LUA_SEQ == msg->value) {
          if ((size_t) len <= arr_cap) {
            ((@(msg_typename)__Sequence*) msg->obj)->size = (size_t) len;
          } else if (!@(msg_prefix)__do_resize(msg, (size_t) len, false)) {
            goto failed;
          }
          lst = ((@(msg_typename)__Sequence*) msg->obj)->data;
        } else {
          goto failed;
        }
      }
      /* copy members */
      idl_lua_msg_t* src = NULL;
      for (int i = 0; i < len; i++) {
        lua_pushinteger(L, i+1);  // push index
        lua_gettable(L, -2);      // pop index, push value
        src = luaL_checkudata(L, -1, "@(msg_metatable)");
        if (src->value >= IDL_LUA_SEQ || !@(msg_typename)__copy(src->obj, lst++)) {
          goto failed;
        }
        lua_pop(L, 1);            // pop value
      }
      done = true;

    } else if (len == 0 && msg->value < IDL_LUA_SEQ) {
      /* dictionary */
      done = rosidl_luacommon_fill_from_table(L);
    }
  }
failed:  // false by default

  lua_pushboolean(L, done);
  return 1;
}

/**
 * Init message methods.
 * \param[inout] L Lua stack.
 */
static void @(msg_prefix)__lconstructor (lua_State* L) {
@{
name_parts = msg_components[2].rsplit('_', 1)
fn_name = name_parts[-1]
}
  /* access via table */
  lua_newtable(L);                       // push table

  /* add metatable */
  lua_createtable(L, 0, @(len(message.constants) + 3));  // push table
  lua_pushcfunction(L, @(msg_prefix)__lnew);  // push function
  lua_setfield(L, -2, "__call");         // pop function, add to table

  /* add constants */
@[for constant in message.constants]@
@[  if isinstance(constant.type, BasicType) and constant.type.typename in NUMERIC_LUA_TYPES]@
@{
type_dict = NUMERIC_LUA_TYPES[constant.type.typename]
}@
  @(type_dict['ifn'])(L, @(constant.value));  // push value
@[  elif isinstance(member.type, BasicType) and constant.type.typename == 'boolean']@
  lua_pushboolean(L, @('true' if constant.value else 'false'));  // push value
@[  elif isinstance(member.type, AbstractString) or isinstance(member.type, AbstractWString)]@
  lua_pushliteral(L, "@(constant.value)");
@[  end if]@
  lua_setfield(L, -2, "@(constant.name)");   // pop value, add to table
@[end for]@

  lua_pushvalue(L, -1);                  // push table
  lua_setfield(L, -2, "__index");        // pop table

  /* type support reference */
  const rosidl_message_type_support_t *ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    @(', '.join(msg_components)));
  lua_pushlightuserdata(L, (void*) ts);
  lua_setfield(L, -2, "_type_support");

  /* metatable */
  lua_pushliteral(L, "@(msg_metatable)");
  lua_setfield(L, -2, "_metatable");

  /* constructor */
  lua_pushcfunction(L, @(msg_prefix)__lnew);  // push function
  lua_setfield(L, -2, "_new");         // pop function, add to table

  lua_setmetatable(L, -2);               // pop table, save as metatable
  lua_setfield(L, -2, "@(fn_name)");  // pop table, save to main table
}

/** Get values */
static const struct luaL_Reg @(msg_prefix)__getters[] = {
@[for name, fn in msg_getters]@
  {"@(name)", @(fn)},
@[end for]@
  {NULL, NULL}
};

/** Set values */
static const struct luaL_Reg @(msg_prefix)__setters[] = {
@[for name, fn in msg_setters]@
  {"@(name)", @(fn)},
@[end for]@
  {NULL, NULL}
};

/** Metamethods */
static const struct luaL_Reg @(msg_prefix)__common[] = {
  {"__gc", @(msg_prefix)__lgc},
  {"__eq", @(msg_prefix)__leq},
  {"__len", @(msg_prefix)__llen},
  {"__tostring", @(msg_prefix)__lstr},
  {"__bnot", @(msg_prefix)__lbnot},
  {"__index", @(msg_prefix)__lindex},
  {"__newindex", @(msg_prefix)__lnewindex},
  {"__call", @(msg_prefix)__lcall},
  {"resize", @(msg_prefix)__lresize},
  {"copy", @(msg_prefix)__lcopy},
  {NULL, NULL}
};

/**
 * Add to library.
 * \param[inout] L Lua stack.
 */
void @(msg_prefix)__add_methods (lua_State* L) {
  /* metatable */
  luaL_newmetatable(L, "@(msg_metatable)");  // push metatable

  /* getters */
  lua_createtable(L, 0, @(len(msg_getters)));  // push table
  luaL_setfuncs(L, @(msg_prefix)__getters, 0);
  lua_setfield(L, -2, "getters");  // pop table

  /* setters */
  lua_createtable(L, 0, @(len(msg_setters)));  // push table
  luaL_setfuncs(L, @(msg_prefix)__setters, 0);
  lua_setfield(L, -2, "setters");  // pop table

  /* common methods */
  luaL_setfuncs(L, @(msg_prefix)__common, 0);

  lua_pop(L, 1);  // pop metatable

  /* add constructor and constants */
  @(msg_prefix)__lconstructor(L);
}

