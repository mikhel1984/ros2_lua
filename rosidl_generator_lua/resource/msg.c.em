
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
    #'rosidl_runtime_c/visibility_control.h',
    include_base + '__struct.h',
    include_base + '__functions.h',
    'rosidl_luacommon/definition.h']
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

@{
msg_components = message.structure.namespaced_type.namespaced_name()
msg_typename = '__'.join(msg_components)
msg_prefix = make_prefix(message)
msg_getters = []
msg_setters = []
msg_metatable = msg_typename + '__mt'
}@

// @(str(msg_components))

@#  constructor
static int @(msg_prefix)__lnew (lua_State* L) {
  // message object
  @(msg_typename)* msg = @(msg_typename)__create();
  if (NULL == msg) {
    luaL_error(L, "failed to create message");
  }
  if (!@(msg_typename)__init(msg)) {
    @(msg_typename)__destroy(msg);
    luaL_error(L, "failed to init message");
  }

  // object wrapper
  idl_lua_msg_t* ptr = lua_newuserdata(L, sizeof(idl_lua_msg_t));
  ptr->obj = msg;
  ptr->value = IDL_LUA_OBJECT;

  // add metamethods
  luaL_getmetatable(L, "@(msg_metatable)");
  lua_setmetatable(L, -2);

  return 1;
}

@#  destructor
static int @(msg_prefix)__lgc (lua_State* L) {
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (IDL_LUA_OBJECT == ptr->value && NULL != ptr->obj) {
    @(msg_typename)__fini(ptr->obj);
    @(msg_typename)__destroy(ptr->obj);
    ptr->obj = NULL;
  }

  return 0;
}

@#  check equality
static int @(msg_prefix)__leq (lua_State* L) {
  // check equality
  lua_getmetatable(L, 1);
  lua_getmetatable(L, 2);
  if (lua_isnil(L, -2) || !lua_rawequal(L, -1, -2)) {
    lua_pushboolean(L, false);
    return 1;
  }
  lua_pop(L, 2);  // remove metatables

  // data
  idl_lua_msg_t* p1 = lua_touserdata(L, 1);
  idl_lua_msg_t* p2 = lua_touserdata(L, 2);

  if (p1->value < IDL_LUA_SEQ && p2->value < IDL_LUA_SEQ) {
    // object or reference
    lua_pushboolean(L, @(msg_typename)__are_equal(p1->obj, p2->obj));
  } else if (p1->value >= IDL_LUA_SEQ && p2->value >= IDL_LUA_SEQ) {
    // lists
    @(msg_typename)__Sequence s1, s2;
    if (p1->value == IDL_LUA_SEQ) {
      s1 = *(@(msg_typename)__Sequence*) p1->obj;
    } else {
      // array, to Sequence object
      s1.data = p1->obj;
      s1.size = s1.capacity = p1->value;
    }
    if (p2->value == IDL_LUA_SEQ) {
      s2 = *(@(msg_typename)__Sequence*) p2->obj;
    } else {
      // array, to Sequence object
      s2.data = p2->obj;
      s2.size = s2.capacity = p2->value;
    }
    lua_pushboolean(L, @(msg_typename)__Sequence__are_equal(&s1, &s2));
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
  if (lua_isnil(L, -2) || !lua_rawequal(L, -1, -2)) {
    // TODO(Mikhel) check type exactly
    lua_pushboolean(L, false);
  }
  lua_pop(L, 2);  // remove metatables

  // data
  idl_lua_msg_t* dst = lua_touserdata(L, 1);
  idl_lua_msg_t* src = lua_touserdata(L, 2);
  bool done = false;

  if (dst->value < IDL_LUA_SEQ && src->value < IDL_LUA_SEQ) {
    // objects
    done = @(msg_typename)__copy(src->obj, dst->obj);
  } else if (IDL_LUA_SEQ == dst->value) {
    // list
    if (IDL_LUA_SEQ == src->value) {
      done = @(msg_typename)__Sequence__copy(src->obj, dst->obj);
    } else if (src->value > IDL_LUA_SEQ) {
      // from array
      @(msg_typename)__Sequence tmp;
      tmp.data = src->obj;
      tmp.size = tmp.capacity = (size_t) src->value;
      done = @(msg_typename)__Sequence__copy(&tmp, dst->obj);
    }
  } else if (dst->value > IDL_LUA_SEQ) {
    // array
    @(msg_typename) *a = dst->obj, *b = NULL;
    if (src->value > IDL_LUA_SEQ && dst->value == src->value) {
      b = src->obj;
    } else if (src->value == IDL_LUA_SEQ) {
      // from list
      @(msg_typename)__Sequence* seq = src->obj;
      if (seq->size == (size_t) dst->value) {
        b = seq->data;
      }
    }
    if (b != NULL) {
      done = true;
      for (int i = 0; i < dst->value; i++) {
        done = @(msg_typename)__copy(b+i, a+i) && done;
      }
    }
  }
  lua_pushboolean(L, done);

  return 1;
}

@# get length
static int @(msg_prefix)__llen (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (ptr->value > IDL_LUA_SEQ) {
    // array
    lua_pushnumber(L, ptr->value);
  } else if (ptr->value == IDL_LUA_SEQ) {
    // list
    @(msg_typename)__Sequence* seq = ptr->obj;
    lua_pushnumber(L, seq->size);
  } else {
    // scalar value
    lua_pushnil(L);
  }

  return 1;
}

@# to string
static int @(msg_prefix)__lstr (lua_State* L)
{
  idl_lua_msg_t* ptr = lua_touserdata(L, 1);
  if (ptr->value > IDL_LUA_SEQ) {
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
  idl_lua_msg_t* ptr = luaL_checkudata(L, 1, "@(msg_metatable)");
  if (ptr->value != IDL_LUA_SEQ) {
    // only list can be resized
    lua_pushboolean(L, false);
    return 1;
  }

  // new length
  lua_Integer len = luaL_checkinteger(L, 2);
  luaL_argcheck(L, len >= 0, 2, "wrong length");
  @(msg_typename)__Sequence* seq = ptr->obj;
  bool done = true;

  if (seq->capacity == 0) {
    // empty object, make new
    done = @(msg_typename)__Sequence__init(seq, len);
  } else if (seq->capacity >= (size_t) len) {
    // memory is enough
    seq->size = (size_t) len;
  } else {
    // allocate new memory and copy data
    @(msg_typename)__Sequence newseq;
    if (@(msg_typename)__Sequence__init(&newseq, len) &&
       @(msg_typename)__Sequence__copy(seq, &newseq))
    {
      // swap
      @(msg_typename)__Sequence tmp = *seq;
      *seq = newseq;
      // remove old
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
setter_ = '_'.join((msg_prefix, '_set', member.name))
}@

// set @(member.name)
static int @(setter_) (lua_State* L) {
  // stack: object, field name, new value
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

  // current object
  // nested type
  luaL_getmetatable(L, "@(nested_metatable)");  // push metatable
  if (lua_isnil(L, -1)) {
    luaL_error(L, "@(nested_type) not found");
  }
  if (lua_getfield(L, -1, "copy") != LUA_TFUNCTION) {  // push function
    luaL_error(L, "no method for object copy");
  }
  // wrap object to call metamethod
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
  // stack: ... function, dst
  lua_pushvalue(L, 3);                        // push argument (duplicate)
  lua_call(L, 2, 1);                          // call copy(L-2, L-1), 2 inputs 1 result

@[  elif isinstance(member.type, AbstractNestedType)]@

  // primitive type sequence
  luaL_getmetatable(L, "@(sequence_metatable(member.type.value_type))");  // push mt
  if (lua_isnil(L, -1)) {
    luaL_error(L, "@(sequence_metatable(member.type.value_type)) not found");
  }
  if (lua_getfield(L, -1, "copy") != LUA_TFUNCTION) {   // push function
    luaL_error(L, "no method for object copy");
  }
  // create object to call metamethod
  idl_lua_msg_t* dst = lua_newuserdata(L, sizeof(idl_lua_msg_t));   // push object
  dst->obj = &(ros_msg->@(member.name));
  dst->value = IDL_LUA_PTR;
  lua_pushvalue(L, -3);                       // push metatable (duplicate)
  lua_setmetatable(L, -2);                    // pop metatable

  // stack: ... function, dst
  lua_pushvalue(L, 3);                        // push argument (duplicate)
  lua_call(L, 2, 1);                          // copy(L-2, L-1)

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
// get @(member.name)
static int @(getter_) (lua_State* L) {
  // stack: object, field name
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
  // return new object
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

@[  elif isinstance(member.type, BasicType) and member.type.typename in ('char', 'octet')]@

  lua_pushinteger(L, ros_msg->@(member.name));

@[  elif isinstance(member.type, AbstractString)]@

  rosidl_runtime_c__String str = ros_msg->@(member.name);
  lua_pushlstring(L, str.data, str.size+1);

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
  idl_lua_msg_t* msg = lua_touserdata(L, 1);

  if (msg->value >= IDL_LUA_SEQ) {
    // object list, same metatable, get by index
    @(msg_typename)* lst = NULL;
    lua_Integer n = luaL_checkinteger(L, 2);
    if (msg->value > IDL_LUA_SEQ) {
      luaL_argcheck(L, 0 < n && n <= msg->value, 2, "out of range");
      lst = msg->obj;
    } else {
      @(msg_typename)__Sequence *seq = msg->obj;
      luaL_argcheck(L, 0 < n && ((size_t) n) <= seq->size, 2, "out_of_range");
      lst = seq->data;
    }
    idl_lua_msg_t* res = lua_newuserdata(L, sizeof(idl_lua_msg_t));  // push obj
    res->obj = &(lst[n-1]);                   // index from 1
    res->value = IDL_LUA_PTR;
    lua_getmetatable(L, 1);                   // push metatable
    lua_setmetatable(L, -2);                  // pop metatable, copy to new object
  } else {
    // nested object, other metatable, get by name
    lua_getmetatable(L, 1);                   // push metatable
    lua_getfield(L, -1, "getters");           // push table
    lua_pushvalue(L, 2);                      // push key (duplicate) 
    lua_gettable(L, -2);                      // pop key, push function
    lua_CFunction fn = lua_tocfunction(L, -1);
    if (NULL == fn) {
      luaL_error(L, "unknown field");
    }
    lua_settop(L, 2);                         // stack: object, key
    fn(L);                                    // push result
  }

  return 1;
}

static int @(msg_prefix)__lnewindex (lua_State* L) {
  idl_lua_msg_t* msg = lua_touserdata(L, 1);

  if (msg->value >= IDL_LUA_SEQ) {
    // object list, same metatable, by index
    @(msg_typename)* lst = NULL;
    lua_Integer n = luaL_checkinteger(L, 2);
    if (msg->value > IDL_LUA_SEQ) {
      luaL_argcheck(L, 0 < n && n <= msg->value, 2, "out of range");
      lst = msg->obj;
    } else {
      @(msg_typename)__Sequence *seq = msg->obj;
      luaL_argcheck(L, 0 < n && ((size_t) n) <= seq->size, 2, "out_of_range");
      lst = seq->data;
    }
    // right part
    idl_lua_msg_t* src = luaL_checkudata(L, 3, "@(msg_metatable)");
    if (src->value >= IDL_LUA_SEQ) {
      luaL_error(L, "different types");
    }
    @(msg_typename)__copy(src->obj, &(lst[n-1]));
  } else {
    // nested object, other metatable, get by name
    lua_getmetatable(L, 1);                   // push metatable
    lua_getfield(L, -1, "setters");           // push table
    lua_pushvalue(L, 2);                      // push key (duplicate)
    lua_gettable(L, -2);                      // pop key, push function
    lua_CFunction fn = lua_tocfunction(L, -1);
    if (NULL == fn) {
      luaL_error(L, "unknown field");
    }
    fn(L);
  }

  return 0;
}

static void @(msg_prefix)__lconstructor (lua_State* L) {
@{
name_parts = msg_components[2].rsplit('_', 1)
fn_name = name_parts[-1]
}
  // access via table
  lua_newtable(L);                       // push table
  
  // add metatable
  lua_createtable(L, 0, @(len(message.constants) + 2));  // push table
  lua_pushcfunction(L, @(msg_prefix)__lnew);  // push function
  lua_setfield(L, -2, "__call");         // pop function, add to table
@[for constant in message.constants]@
@{
type_dict = NUMERIC_LUA_TYPES[constant.type.typename]
}@  
  @(type_dict['ifn'])(L, @(constant.value));  // push value
  lua_setfield(L, -2, "@(constant.name)");   // pop value, add to table
@[end for]@

  lua_pushvalue(L, -1);                  // push table
  lua_setfield(L, -2, "__index");        // pop table
  
  lua_setmetatable(L, -2);               // pop table, save as metatable  
  lua_setfield(L, -2, "@(fn_name)");  // pop table, save to main table
}

// get values
static const struct luaL_Reg @(msg_prefix)__getters[] = {
@[for name, fn in msg_getters]@
  {"@(name)", @(fn)},
@[end for]@
  {NULL, NULL}
};

// set values
static const struct luaL_Reg @(msg_prefix)__setters[] = {
@[for name, fn in msg_setters]@
  {"@(name)", @(fn)},
@[end for]@
  {NULL, NULL}
};


static const struct luaL_Reg @(msg_prefix)__common[] = {
  {"__gc", @(msg_prefix)__lgc},
  {"__eq", @(msg_prefix)__leq},
  {"__len", @(msg_prefix)__llen},
  {"__tostring", @(msg_prefix)__lstr},
  {"__index", @(msg_prefix)__lindex},
  {"__newindex", @(msg_prefix)__lnewindex},
  {"resize", @(msg_prefix)__lresize},
  {"copy", @(msg_prefix)__lcopy},
  {NULL, NULL}
};


void @(msg_prefix)__add_methods (lua_State* L) {
  // metatable
  luaL_newmetatable(L, "@(msg_metatable)");  // push metatable

  // getters
  lua_createtable(L, 0, @(len(msg_getters)));  // push table
  luaL_setfuncs(L, @(msg_prefix)__getters, 0);
  lua_setfield(L, -2, "getters");  // pop table

  // setters
  lua_createtable(L, 0, @(len(msg_setters)));  // push table
  luaL_setfuncs(L, @(msg_prefix)__setters, 0);
  lua_setfield(L, -2, "setters");  // pop table

  // common methods
  luaL_setfuncs(L, @(msg_prefix)__common, 0);

  lua_pop(L, 1);  // pop metatable

  // add constructor and constants
  @(msg_prefix)__lconstructor(L);
}

