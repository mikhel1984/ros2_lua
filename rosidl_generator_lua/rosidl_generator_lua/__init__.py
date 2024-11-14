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
    'float': {'min': 'FLT_MIN', 'max': 'FLT_MAX', 'var': 'lua_Number', 'fn': 'luaL_checknumber', 'ifn': 'lua_pushnumber', 'ctype': 'float'},
    'double': {'min': 'DBL_MIN', 'max': 'DBL_MAX', 'var': 'lua_Number', 'fn': 'luaL_checknumber', 'ifn': 'lua_pushnumber', 'ctype': 'double'},
    'int8': {'min': 'INT8_MIN', 'max': 'INT8_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushinteger', 'ctype': 'int8_t'},
    'uint8': {'min': '0', 'max': 'UINT8_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber', 'ctype': 'uint8_t'},
    'int16': {'min': 'INT16_MIN', 'max': 'INT16_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber', 'ctype': 'int16_t'},
    'uint16': {'min': '0', 'max': 'UINT16_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber', 'ctype': 'uint16_t'},
    'int32': {'min': 'INT32_MIN', 'max': 'INT32_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber', 'ctype': 'int32_t'},
    'uint32': {'min': '0', 'max': 'UINT32_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber', 'ctype': 'uint32_t'},
    'int64': {'min': 'INT64_MIN', 'max': 'INT64_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber', 'ctype': 'int64_t'},
    'uint64': {'min': '0', 'max': 'UINT64_MAX', 'var': 'lua_Integer', 'fn': 'luaL_checkinteger', 'ifn': 'lua_pushnumber', 'ctype': 'uint64_t'},
}


def sequence_metatable(type_):
    if type_.typename in ('char', 'octet'):
        return "ROS2.rosidl_sequence.int8"
    if isinstance(type_, AbstractGenericString):
        return "ROS2.rosidl_sequence.String"
    return "ROS2.rosidl_sequence." + type_.typename


def generate_lua(generator_arguments_file, typesupport_impls):
    print("CALL LUA!!!", file=sys.stderr)
    mapping = {
        'idl.lua.em': '%s.lua',
        'idl.c.em': '%s.c',
    }
    generated_files = generate_files(generator_arguments_file, mapping)
    
    args = read_generator_arguments(generator_arguments_file)
    template_dir = args['template_dir']
        
    
    modules = {}
    idl_content = IdlContent()
    for idl_tuple in args.get('idl_tuples', []):
        idl_parts = idl_tuple.rsplit(':', 1)
        assert len(idl_parts) == 2

        idl_rel_path = pathlib.Path(idl_parts[1])
        idl_stems = modules.setdefault(str(idl_rel_path.parent), set())
        idl_stems.add(idl_rel_path.stem)

        locator = IdlLocator(*idl_parts)
        idl_file = parse_idl_file(locator)
        idl_content.elements += idl_file.content.elements
    
    obj_list = [
        ('msg', idl_content.get_elements_of_type(Message)),
        ('srv', idl_content.get_elements_of_type(Service)),
        ('action', idl_content.get_elements_of_type(Action)),
    ]
    
    
#    for message in idl_content.get_elements_of_type(Message):
#        msg_types.append('msg')
#        break
#        print(message.structure.namespaced_type.name, file=sys.stderr)
#        for member in message.structure.members:
#            print(member.name, member.type, file=sys.stderr)    


    
    lib_templates = [('msg_lib.c.em', 'msg', 'msg_lib.txt')]
    latest_target_timestamp = get_newest_modification_time(args['target_dependencies'])
    for msg_type, idl_group in obj_list:
        if not idl_group:
            continue  
    #for template_file, interface, out_name in lib_templates:
        template_file = msg_type + '_lib.c.em'
        out_name = msg_type + '_lib.txt'
        package_name = args['package_name']
        data = {
            'package_name': args['package_name'],
            'content': idl_group, # idl_content,
        }
        generated_file = os.path.join(
            args['output_dir'], msg_type, out_name)
        template = os.path.join(template_dir, template_file)
        expand_template(
            template, data, generated_file,
            minimum_timestamp=latest_target_timestamp)
        generated_files.append(generated_file)
        print('build file!!!', file=sys.stderr)

    
    return generated_files

