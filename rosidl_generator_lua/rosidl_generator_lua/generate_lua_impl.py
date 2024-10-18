# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
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

from ast import literal_eval
import keyword
import os
import pathlib
import sys

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import generate_files
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import Action
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import CHARACTER_TYPES
from rosidl_parser.definition import FLOATING_POINT_TYPES
from rosidl_parser.definition import IdlContent
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import INTEGER_TYPES
from rosidl_parser.definition import Message
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import Service
from rosidl_parser.parser import parse_idl_file


NUMERIC_LUA_TYPES = {
    'float': {'min': 'FLT_MIN', 'max': 'FLT_MAX', 'var': 'lua_Number', 'fn': 'luaL_checknumber', 'ifn': 'lua_pushnumber'},
    'double': {'min': 'DBL_MIN', 'max': 'DBL_MAX', 'var': 'lua_Number', 'fn': 'luaL_checknumber', 'ifn': 'lua_pushnumber'},
    'int8': {'min': 'INT8_MIN', 'max': 'INT8_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushinteger'},
    'uint8': {'min': '0', 'max': 'UINT8_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber'},
    'int16': {'min': 'INT16_MIN', 'max': 'INT16_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber'},
    'uint16': {'min': '0', 'max': 'UINT16_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber'},
    'int32': {'min': 'INT32_MIN', 'max': 'INT32_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber'},
    'uint32': {'min': '0', 'max': 'UINT32_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber'},
    'int64': {'min': 'INT64_MIN', 'max': 'INT64_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber'},
    'uint64': {'min': '0', 'max': 'UINT64_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber'},
}


def sequence_metatable(type_):
    if type_.typename in ('char', 'octet'):
        return "ROS2.rosidl_sequence.int8"
    if isinstance(type_, AbstractGenericString):
        return "ROS2.rosidl_sequence.String"
    return "ROS2.rosidl_sequence" + type_.typename


def generate_lua(generator_arguments_file, typesupport_impls):
    print("CALL LUA!!!", file=sys.stderr)
    mapping = {
        'idl.lua.em': '%s.lua',
        'idl.c.em': '%s.c',
    }
    generated_files = generate_files(generator_arguments_file, mapping)

#    args = read_generator_arguments(generator_arguments_file)
#    package_name = args['package_name']

#    # expand init modules for each directory
#    modules = {}
#    idl_content = IdlContent()
#    for idl_tuple in args.get('idl_tuples', []):
#        idl_parts = idl_tuple.rsplit(':', 1)
#        assert len(idl_parts) == 2

#        idl_rel_path = pathlib.Path(idl_parts[1])
#        idl_stems = modules.setdefault(str(idl_rel_path.parent), set())
#        idl_stems.add(idl_rel_path.stem)

#        locator = IdlLocator(*idl_parts)
#        idl_file = parse_idl_file(locator)
#        idl_content.elements += idl_file.content.elements

#    def print_warning_if_reserved_keyword(member_name, interface_type, interface_name):
#        if (keyword.iskeyword(member.name)):
#            print(
#                "Member name '{}' in the {} '{}' is a "
#                'reserved keyword in Python and is not supported '
#                'at the moment. Please use a different name.'
#                .format(member_name, interface_type, interface_name),
#                file=sys.stderr)

#    for message in idl_content.get_elements_of_type(Message):
#        for member in message.structure.members:
#            print_warning_if_reserved_keyword(
#                member.name, 'message',
#                message.structure.namespaced_type.name)

#    for service in idl_content.get_elements_of_type(Service):
#        for member in service.request_message.structure.members:
#            print_warning_if_reserved_keyword(
#                member.name, 'service request',
#                service.namespaced_type.name)
#        for member in service.response_message.structure.members:
#            print_warning_if_reserved_keyword(
#                member.name, 'service response',
#                service.namespaced_type.name)

#    for action in idl_content.get_elements_of_type(Action):
#        for member in action.goal.structure.members:
#            print_warning_if_reserved_keyword(
#                member.name, 'action goal',
#                action.namespaced_type.name)
#        for member in action.feedback.structure.members:
#            print_warning_if_reserved_keyword(
#                member.name, 'action feedback',
#                action.namespaced_type.name)
#        for member in action.result.structure.members:
#            print_warning_if_reserved_keyword(
#                member.name, 'action result',
#                action.namespaced_type.name)

#    for subfolder in modules.keys():
#        with open(os.path.join(args['output_dir'], subfolder, '__init__.py'), 'w') as f:
#            module_names = {}
#            for idl_stem in modules[subfolder]:
#                module_names[idl_stem] = '_' + \
#                    convert_camel_case_to_lower_case_underscore(idl_stem)
#            # sorting after lower case conversion to get true order
#            for module_name, idl_stem in \
#                    sorted((value, key) for (key, value) in module_names.items()):
#                f.write(
#                    f'from {package_name}.{subfolder}.{module_name} import '
#                    f'{idl_stem}  # noqa: F401\n')

#    # expand templates per available typesupport implementation
#    template_dir = args['template_dir']
#    type_support_impl_by_filename = {
#        '_%s_s.ep.{0}.c'.format(impl): impl for impl in typesupport_impls
#    }
#    mapping_msg_pkg_extension = {
#        os.path.join(template_dir, '_idl_pkg_typesupport_entry_point.c.em'):
#        type_support_impl_by_filename.keys(),
#    }

#    for template_file in mapping_msg_pkg_extension.keys():
#        assert os.path.exists(template_file), 'Could not find template: ' + template_file

#    latest_target_timestamp = get_newest_modification_time(args['target_dependencies'])

#    for template_file, generated_filenames in mapping_msg_pkg_extension.items():
#        for generated_filename in generated_filenames:
#            package_name = args['package_name']
#            data = {
#                'package_name': args['package_name'],
#                'content': idl_content,
#                'typesupport_impl': type_support_impl_by_filename.get(generated_filename, ''),
#            }
#            generated_file = os.path.join(
#                args['output_dir'], generated_filename % package_name
#            )
#            expand_template(
#                template_file, data, generated_file,
#                minimum_timestamp=latest_target_timestamp)
#            generated_files.append(generated_file)

    return generated_files



#def primitive_value_to_py(type_, value):
#    assert value is not None

#    if isinstance(type_, AbstractGenericString):
#        return quoted_string(value)

#    assert isinstance(type_, BasicType)

#    if type_.typename == 'boolean':
#        return 'True' if value else 'False'

#    if type_.typename in INTEGER_TYPES:
#        return str(value)

#    if type_.typename == 'char':
#        return repr('%c' % value)

#    if type_.typename == 'octet':
#        return repr(bytes([value]))

#    if type_.typename in FLOATING_POINT_TYPES:
#        return '%s' % value

#    assert False, "unknown primitive type '%s'" % type_.typename


#def constant_value_to_py(type_, value):
#    assert value is not None

#    if isinstance(type_, BasicType):
#        if type_.typename == 'boolean':
#            return 'True' if value else 'False'

#        if type_.typename in INTEGER_TYPES:
#            return str(value)

#        if type_.typename == 'char':
#            return repr('%c' % value)

#        if type_.typename == 'octet':
#            return repr(bytes([value]))

#        if type_.typename in FLOATING_POINT_TYPES:
#            return '%s' % value

#    if isinstance(type_, AbstractGenericString):
#        return quoted_string(value)

#    assert False, "unknown constant type '%s'" % type_


#def quoted_string(s):
#    s = s.replace('\\', '\\\\')
#    # strings containing single quote but no double quotes can be wrapped in
#    # double quotes without escaping
#    if "'" in s and '"' not in s:
#        return '"%s"' % s
#    # all other strings are wrapped in single quotes, if necessary with escaped
#    # single quotes
#    s = s.replace("'", "\\'")
#    return "'%s'" % s


#def get_python_type(type_):
#    if isinstance(type_, NamespacedType):
#        return type_.name

#    if isinstance(type_, AbstractGenericString):
#        return 'str'

#    if isinstance(type_, AbstractNestedType):
#        if isinstance(type_.value_type, BasicType) and type_.value_type.typename == 'octet':
#            return 'bytes'

#        if (
#            isinstance(type_.value_type, BasicType) and
#            type_.value_type.typename in CHARACTER_TYPES
#        ):
#            return 'str'

#    if isinstance(type_, BasicType) and type_.typename == 'boolean':
#        return 'bool'

#    if isinstance(type_, BasicType) and type_.typename == 'octet':
#        return 'bytes'

#    if isinstance(type_, BasicType) and type_.typename in INTEGER_TYPES:
#        return 'int'

#    if isinstance(type_, BasicType) and type_.typename in FLOATING_POINT_TYPES:
#        return 'float'

#    if isinstance(type_, BasicType) and type_.typename in CHARACTER_TYPES:
#        return 'str'

#    assert False, "unknown type '%s'" % type_
