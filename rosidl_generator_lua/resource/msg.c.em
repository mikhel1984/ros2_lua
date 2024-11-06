
@{
from rosidl_generator_lua import NUMERIC_LUA_TYPES, sequence_metatable
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME
from rosidl_parser.definition import NamespacedType

include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)

header_files = [
    'lua.h',
    'lauxlib.h',
    'limits.h',
    'stdint.h',
    'stdbool.h',
    'rosidl_runtime_c/visibility_control.h',
    include_base + '__struct.h',
    include_base + '__functions.h']
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
// @
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
// @
@[    else]@
@{include_directives.add(nested_header)}@
@[    end if]@
#include "@(nested_header)"
@[  end for]@
// end nested array functions include
@[end if]@

#include "msg.h" // rename
@{
msg_typename = '__'.join(message.structure.namespaced_type.namespaced_name())
msg_prefix = '__'.join(message.structure.namespaced_type.namespaces + [convert_camel_case_to_lower_case_underscore(message.structure.namespaced_type.name)])
msg_getters = []
msg_setters = []
}@

@#  constructor
static int @(msg_prefix)__lnew (lua_State* L) {
  idl_lua_msg_t* ptr = lua_newuserdata(L, sizeof(idl_lua_msg_t));
  ptr->value = IDL_LUA_OBJECT;

  @(msg_typename)* msg = @(msg_typename)__create();
  if (NULL == msg) {
    luaL_error(L, "failed to create message");
  }
  @(msg_typename)__init(msg);
  ptr->obj = msg;

  luaL_getmetatable(L, "@(msg_typename)_mt");
  lua_setmetatable(L, -2);

  return 1;
}

@#  destructor
static int @(msg_prefix)__lgc (lua_State* L) {
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (IDL_LUA_OBJECT == ptr->value) {
    @(msg_typename)__fini(ptr->obj);
    @(msg_typename)__destroy(ptr->obj);
  }

  return 0;
}

@#  check equality
static int @(msg_prefix)__leq (lua_State* L) {
  // check equality
  lua_getmetatable(L, 1);
  lua_getmetatable(L, 2);
  if (lua_isnil(L, -1) || !lua_rawequal(L, -1, -2)) {
    lua_pushboolean(L, false);
    return 1;
  }
  lua_pop(L, 2);

  // data
  idl_lua_msg_t* p1 = lua_touserdata(L, 1);
  idl_lua_msg_t* p2 = lua_touserdata(L, 2);

  if (p1->value < IDL_LUA_SEQ && p2->value < IDL_LUA_SEQ) {
    // objects
    lua_pushboolean(@(msg_typename)__are_equal(p1->obj, p2->obj));
  } else if (p1->value >= IDL_LUA_SEQ && p2->value >= IDL_LUA_SEQ) {
    // lists
    @(msg_typename)__Sequence s1, s2;
    if (p1->value == IDL_LUA_SEQ) {
      s1 = *(@(msg_typename)__Sequence*) p1->obj;
    } else {
      s1.data = p1->obj;
      s1.size = s1.capacity = p1->value;
    }
    if (p2->value == IDL_LUA_SEQ) {
      s2 = *(@(msg_typename)__Sequence*) p2->obj;
    } else {
      s2.data = p2->obj;
      s2.size = s2.capacity = p2->value;
    }
    lua_pushboolean(@(msg_typename)__Sequence__are_equal(&s1, &s2));
  } else {
    lua_pushboolean(L, false);
  }

  return 1;
}

@# copy value
static int @(msg_prefix)__lcopy (lua_State* L) {
  // check equality
  lua_getmetatable(L, 1);
  lua_getmetatable(L, 2);
  if (lua_isnil(L, -1) || !lua_rawequal(L, -1, -2)) {
    // TODO(Mikhel) check type exactly
    lua_pushboolean(L, false);
  }
  lua_pop(L, 2);

  // data
  idl_lua_msg_t* dst = lua_touserdata(L, 1);
  idl_lua_msg_t* src = lua_touserdata(L, 2);
  bool done = false;

  if (dst->value < IDL_LUA_SEQ && src->value < IDL_LUA_SEQ) {
    // objects
    done = @(msg_typename)__copy(src->obj, dst->obj);
  } else if (dst->value == IDL_LUA_SEQ) {
    // list
    if (src->value == IDL_LUA_SEQ) {
      done = @(msg_typename)__Sequence__copy(src->obj, dst->obj);
    } else if (src->value > 0) {
      @(msg_typename)__Sequence tmp;
      tmp.data = src->obj;
      tmp.size = tmp.capacity = (size_t) src->value;
      done = @(msg_typename)__Sequence__copy(&tmp, dst->obj);
    }
  } else if (dst->value > 0) {
    // array
    @(msg_typename)__Sequence *a = dst->obj, *b = NULL;
    if (src->value > 0 && dst->value == src->value) {
      b = src->obj;
    } else if (src->value == IDL_LUA_SEQ) {
      @(msg_typename)__Sequence* seq = src->obj;
      if (seq->size == (size_t) dst->value) {
        b = seq->data;
      }
    }
    if (b != NULL) {
      for (int i = 0; i < dst->value; i++) {
        @(msg_typename)__copy(b, a);
      }
      done = true;
    }
  }
  lua_pushboolean(L, done);

  return 1;
}

@# get length
static int @(msg_prefix)__llen (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (ptr->value > 0) {
    lua_pushnumber(L, ptr->value);
  } else if (ptr->value == IDL_LUA_SEQ) {
    @(msg_typename)__Sequence* seq = ptr->obj;
    lua_pushnumber(L, seq->size);
  } else {
    lua_pushnil(L);
  }

  return 1;
}

@# to string
static int @(msg_prefix)__lstr (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (ptr->value > 0) {
    lua_pushfstring(L, "@(msg_typename) array of size %d", ptr->value);
  } else if (ptr->value == IDL_LUA_SEQ) {
    @(msg_typename)__Sequence* seq = ptr->obj;
    lua_pushfstring(L, "@(msg_typename) sequence of size %d", seq->size);
  } else {
    lua_pushfstring(L, "@(msg_typename)");
  }

  return 1;
}

@# resize sequence
static int @(msg_prefix)__lresize (lua_State* L)
{
  idl_lua_msg_t* ptr = luaL_checkudata(L, 1, "@(msg_typename)_mt");
  if (ptr->value != IDL_LUA_SEQ) {
    lua_pushboolean(L, false);
    return 1;
  }

  lua_Integer len = luaL_checkinteger(L, 2);
  luaL_argcheck(L, len >= 0, 2, "wrong length");
  @(msg_typename)__Sequence* seq = ptr->obj;
  bool done = true;

  if (seq->capacity == 0) {
    done = @(msg_typename)__Sequence__init(seq, len);
  } else if (seq->capacity >= (size_t) len) {
    seq->size = (size_t) len;
  } else {
    @(msg_typename)__Sequence newseq;
    if (@(msg_typename)__Sequence__init(&newseq, len) &&
       @(msg_typename)__Sequence__copy(seq, &newseq))
    {
      @(msg_typename)__Sequence tmp = *seq;
      *seq = newseq;
      @(msg_typename)__Sequence__fini(&tmp);
    } else {
      done = false;
    }
  }
  lua_pushboolean(L, done);

  return 1;
}


@#  setters
@[for member in message.structure.members]@
@[  if len(message.structure.members) == 1 and member.name == EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME]@
@[    continue]@
@[  end if]@
@{
setter_ = '_'.join((msg_typename, member.name, 'set'))
}@

// @(member.name)
static int @(setter_) (lua_State* L) {
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  msg_typename* ros_msg = ptr->obj;
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
}@
@[  if isinstance(type_, NamespacedType)]@
@{
nested_type = '__'.join(type_.namespaced_name())
}@

  lua_getmetatable(L, "@(msg_typename)_mt");
  lua_getfield(L, -1, "copy");  // get function
  idl_lua_msg_t* dst = lua_newuserdata(L, sizeof(idl_lua_msg_t));  // new object
  dst->obj = &(ros_message->@(member.name));
  dst->value = IDL_LUA_PTR;
  lua_setmetatable(L, -3);  // src
  lua_pushvalue(L, 3);
  lua_call(L, 2, 1);  // copy(L-2, L-1)

@[  elif isinstance(member.type, AbstractNestedType)]@

  luaL_getmetatable(L, @(sequence_metatable(member.type.value_type)));
  lua_getfield(L, -1, "copy");  // get function
  idl_lua_msg_t* dst = lua_newuserdata(L, sizeof(idl_lua_msg_t));  // new object
  dst->obj = &(ros_message->@(member.name));
  dst->value = IDL_LUA_PTR;
  lua_setmetatable(L, -3);  // src
  lua_pushvalue(L, 3);
  lua_call(L, 2, 1);  // copy(L-2, L-1)

@[  elif isinstance(member.type, BasicType) and member.type.typename in ('char', 'octet')]@

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
  if (value < @(type_dict['min']) || value > @(type_dict['max'])) {
    luaL_error(L, "value out of range");
  }
  ros_msg->@(member.name) = value;

@[  elif isinstance(member.type, AbstractString)]@

  const char* value = luaL_checkstring(L, 3);
  rosidl_runtime_c__String__assign(&ros_message->@(member.name), value);

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
getter_ = '_'.join((msg_typename, member.name, 'get'))
}@

// @(member.name)
static int @(getter_) (lua_State* L) {
  idl_lua_msg_t* ptr = luaL_checkudata(L, 1, '@(msg_typename)_mt');
  msg_typename* ros_msg = ptr->obj;

@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
}@

@[  if isinstance(type_, NamespacedType)]@
@{
nested_type = '__'.join(type_.namespaced_name())
mtbl = '__'.join(type_.namespaces + [convert_camel_case_to_lower_case_underscore(type_.name)]) + '_mt'
}@

  // @(mtbl)
  idl_lua_msg_t* ptr = lua_newuserdata(L, sizeof(idl_lua_msg_t));

@[    if isinstance(member.type, AbstractNestedType)]@
@[      if isinstance(member.type, AbstractSequence)]@

  ptr->obj = &(ros_msg->@(member.name));
  ptr->value = IDL_LUA_SEQ;

@[      else]@

  ptr->obj = ros_msg->@(member.name);
  ptr->value = // set size!!!

@[      end if]@
@[    else]@

  ptr->obj = &(ros_msg->@(member.name));
  ptr->value = IDL_LUA_PTR;

@[    end if]@

  luaL_getmetatable(L, "@(mtbl)");
  lua_setmetatable(L, -2);

@[  elif isinstance(member.type, AbstractNestedType)]@

  idl_lua_msg_t* ptr = lua_newuserdata(L, sizeof(idl_lua_msg_t));

@[    if isinstance(member.type, AbstractSequence)]@

  ptr->obj = &(ros_msg->@(member.name));
  ptr->value = IDL_LUA_SEQ;

@[    else]@

  ptr->obj = ros_msg->@(member.name);
  ptr->value = // set size !!!

@[    end if]@
@[    if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename in ('char', 'octet')]@

  luaL_getmetatable(L, @(sequence_metatable(member.type.value_type)));
  lua_setmetatable(L, -2);
@[    end if]@

@[  elif isinstance(member.type, BasicType) and member.type.typename in ('char', 'octet')]@

  lua_pushinteger(L, &(ros_msg->@(member.name)));

@[  elif isinstance(member.type, AbstractString)]@

  lua_pushstring(L, ros_msg->@(member.name));

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


static int @(msg_prefix)__lindex (lua_State* L) {
  lua_getmetatable(L, 1);
  idl_lua_msg_t* msg = lua_touserdata(L, 1);

  if (msg->value >= IDL_LUA_SEQ) {
    // by index
    some_obj* lst = NULL;
    lua_Integer n = luaL_checkinteger(L, 2);
    if (msg->value > IDL_LUA_SEQ) {
      luaL_argcheck(L, 0 < n && n <= msg->value, 2, "out of range");
      lst = msg->obj;
    } else {
      some_obj_type_Sequence *seq = msg->obj;
      luaL_argcheck(L, 0 < n && ((size_t) n) <= seq->size, 2, "out_of_range");
      lst = msg->data;
    }
    idl_lua_msg_t* res = lua_newuserdata(L, sizeof(idl_lua_msg_t));
    res->obj = &(lst[n-1]);
    res->value = IDL_LUA_PTR;
  } else {
    // by name
    lua_getfield(L, -1, "_get");
    lua_pushvalue(L, 2);  // duplicate key
    lua_gettable(L, -2);  // find field
    lua_CFunction fn = lua_tocfunction(L, -1);
    if (NULL == fn) {
      luaL_error(L, "unknown field");
    }
    lua_settop(L, 2);
    fn(L);
  }

  return 1;
}


static int @(msg_prefix)__lnewindex (lua_State* L) {
  lua_getmetatable(L, 1);
  idl_lua_msg_t* msg = lua_touserdata(L, 1);

  if (msg->value >= IDL_LUA_SEQ) {
    // by index
    some_obj* lst = NULL;
    lua_Integer n = luaL_checkinteger(L, 2);
    if (msg->value > IDL_LUA_SEQ) {
      luaL_argcheck(L, 0 < n && n <= msg->value, 2, "out of range");
      lst = msg->obj;
    } else {
      some_obj_type_Sequence *seq = msg->obj;
      luaL_argcheck(L, 0 < n && ((size_t) n) <= seq->size, 2, "out_of_range");
      lst = msg->data;
    }
    idl_lua_msg_t* res = lua_newuserdata(L, sizeof(idl_lua_msg_t));
    res->obj = &(lst[n-1]);
    res->value = IDL_LUA_PTR;
  } else {
    // by name
    lua_getfield(L, -1, "_set");
    lua_pushvalue(L, 2);  // duplicate key
    lua_gettable(L, -2);  // find field
    lua_CFunction fn = lua_tocfunction(L, -1);
    if (NULL == fn) {
      luaL_error(L, "unknown field");
    }
    lua_settop(L, 2);
    fn(L);
  }

  return 0;
}


