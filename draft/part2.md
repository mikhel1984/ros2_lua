# Генерация ROS2 сообщений

Продолжаем разбираться с тем, что нужно сделать, чтобы портировать ROS2 на свой любимый язык программирования. На этот раз рассмотрим работу с сообщениями. 

Обмен данными играет в ROS2 ключевую роль. К счастью, практически все задачи, связанные с передачей и приемом сообщений берут на себя библиотеки [rcl](https://github.com/ros2/rcl) и [rmw](https://github.com/ros2/rmw), нам "всего лишь" необходимо обеспечить возможность их создания и обработки. 

Структура сообщения в ROS2 описывается в файле с расширением _msg_ или _idl_. При сборке пакета запускается генератор, который преобразует исходные сообщения в C библиотеку, которая, в свою очередь, используется при генерации библиотек для C++ и Python. На выходе получаем исходный код на C/C++/Python и скомпилированные файлы динамических библиотек. Рассмотрим подробнее, как эта генерация устроена.

## Представление сообщений в C

Прежде чем переходить к другим языкам полезно посмотреть, какую структуру имеют сообщения на C. Вообще, если ваш язык программирования предоставляет возможность взаимодействовать с полями C структур напрямую, вам очень повезло. В противном случае, придётся писать функции для доступа к данным.

Генератор C кода формирует файлы нескольких типов. Непосредственно структура сообщений описана в файле *тип__struct.h*. В файлах *тип__functions* хранятся функции для инициализации, освобождения, копирования, проверки равенства. В *тип__description.c* содержится детальная информация о структуре сообщений, а в *тип__type_support* - функции генерации сообщений о событиях.

Поля C структуры расположены в том же порядке, в каком они описаны в файле msg. Для элементарных типов (таких как uint8, float64, bool) используется соответствующий тип из C. Для объекта формируется отдельная структура, которая затем добавляется в сообщение. Если поле представляет собой статический массив, т.е. число элементов известно заранее, аналогичный массив закладывается и в C структуру. Динамический массив описывается указателем на выделяемую память, текущим числом элементов и размером выделенной памяти. Для примитивных типов и строк такие структуры заранее определены, поэтому генератор просто добавляет в хэдеры библиотеку *rosidl_runtime_c*.


## rosidl_generator

Генератор реализуется в виде отдельного ROS2 пакета на Python и включает в себя следующие элементы.

### Папка resource

Здесь содержатся шаблоны для функций сериализации и десериализации сообщений под конкретный язык программирования. Они имеют расширение *em* и синтаксически являются макросами над языком Python, позволяющими комбинироавть сырой текст с исполняемым кодом. Можно обнаружить 4 типа таких макросов: для управления генерацией кода (@[ ]), определения промежуточных переменных (@{ }), подстановки значения в генерируемый текст (@( )) и комментариев (@#).
```
@# это строчный комментарий

@# генерация в цикле или по условию
@[инициализация блока]@
текст программы
@[  опциональное промежуточное условие, со смещением]@
альтернативный текст программы
@[конец блока]@

@# объявление переменных, функций, прочий код на Python
@{
var = 42
}@

продолжение программы, значение var равно @(var)
```

Основная логика преобразования сообщений определена в файле msg.c.em (название может быть произвольным), который служит для генерации C кода. Здесь можно реализовать все необходимые функции для работы с сообщениями, но прежде всего необходимы методы для сериализации и десериализации. Они используют следующий шаблон.
```python
@{
from rosidl_generator_lua import NUMERIC_LUA_TYPES, sequence_metatable, make_prefix
from rosidl_parser.definition import EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME
}@

@# формирование C структуры из Lua сообщения
@[for member in message.structure.members]@
@[  if len(message.structure.members) == 1 and member.name == EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME]@
@[    continue]@
@[  end if]@
@{
msg_prefix = make_prefix(message)
setter_ = '_'.join((msg_prefix, '_set', member.name))
}@

static int @(setter_) (lua_State* L) {

  // подготовительные операции ...

@# генерация наименования типа
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
}@

  // значение считывается из поля с именем
  // @(member.name)

@# комплексный тип данных
@[  if isinstance(type_, NamespacedType)]@
@#    массив
@[    if isinstance(member.type, AbstractNestedType)]@
@#      динамический массив
@[      if isinstance(member.type, AbstractSequence)]@

  // копирование данных в динамический массив

@#      статический массив
@[      else]@

  // копирование данных в массив

@[      end if]@
@#    объект
@[    else]@

  // копирование объекта

@[    end if]@
@#  последовательность примитивных типов
@[  elif isinstance(member.type, AbstractNestedType)]@
@#    динамический массив
@[    if isinstance(member.type, AbstractSequence)]@

  // копирование данных в динамический массив

@#    статический массив
@[    else]@

  // копирование данных в массив

@[    end if]@
@#  литерал
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'char']@

  // копирование одиночного символа

@#  логическая переменная
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'boolean']@

  // копирование логической переменной

@#  число
@[  elif isinstance(member.type, BasicType) and member.type.typename in NUMERIC_LUA_TYPES]@
@{
type_dict = NUMERIC_LUA_TYPES[member.type.typename]
}@
@#    проверка беззнакового числа
@[    if member.type.typename.startswith('u') ]@

  // проверка диапазона беззнаковых чисел

@[    else]@

  // проверка диапазона с учетом знака

@[    end if]@

   // копирование значения

@#  строка в 8-битной кодировке
@[  elif isinstance(member.type, AbstractString)]@

  // копирование строки

@#  строка в 16-битной кодировке
@[  elif isinstance(member.type, AbstractWString)]@

  // копирование строки

@[  else]@
@{
assert False, ("unknown type " + member.type.typename)
}@
@[  end if]@
  return 0;
}
@[end for]@
```
Список полей сообщения определен в переменной *message.structure.members*. Для каждого поля шаблон проверяет тип переменной, а также является ли она примитивом, объектом или массивом. В последнем случае дополнительно проверяется тип массива (статический или динамический), и какие элементы в нём содержатся (примитивы или объекты). Исходя из этого должна быть реализована логика преобразования в C и обратно.

Генерацией файлов для заданного пакета управляет шаблон *idl.c.em*. Он итеративно вызывает функцию *TEMPLATE* для каждого найденного сообщения.
```python
@{
from rosidl_parser.definition import Message
include_directives = set()
}@
@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
    'msg.c.em',
    package_name=package_name, interface_path=interface_path,
    message=message, include_directives=include_directives)
}@
@[end for]@
```
В случае сервисов генерируются 2 сообщения, соответствующие полям *request_message* и *response_message*. Для action сервисов число сообщений увеличивается до 8 (поля *goal*, *result*, *feedback*, *send_goal_service.request_message*, *send_goal_message.response_message*, *get_result_service.request_message*, *get_result_service.response_message*, *action.feedback_message*).

В папке *resource* можно найти и другие *em* файлы. Они служат для оборачивания сгенерированных сообщений в одну или несколько динамических библиотек.


### Папка rosidl_generator

Скрипт *\_\_init.py\_\_* определяет функцию для запуска генерации сообщений, а также вспомогательный функционал, используемый внутри генератора.
```python
import os
import pathlib

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import generate_files
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import IdlContent
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import Message
from rosidl_parser.definition import Service
from rosidl_parser.definition import Action
from rosidl_parser.parser import parse_idl_file

# переменные и функции для обработки числовых полей сообщений
NUMERIC_LUA_TYPES = {
    'float': {'min': 'FLT_MIN', 'max': 'FLT_MAX', 'var': 'lua_Number', 'fn': 'luaL_checknumber',
              'ifn': 'lua_pushnumber', 'ctype': 'float'},
    # etc.
}

# основная функция для генерации C кода библиотеки
def generate_lua(generator_arguments_file, typesupport_impls):
    mapping = {'idl.c.em': '%s.c'}
    generated_files = generate_files(generator_arguments_file, mapping)

    args = read_generator_arguments(generator_arguments_file)
    template_dir = args['template_dir']

    # разбор idl файлов
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

    # разделяем по типам в соответствии с реализованными шаблонами
    obj_list = [
        ('msg', idl_content.get_elements_of_type(Message)),
        ('srv', idl_content.get_elements_of_type(Service)),
        ('action', idl_content.get_elements_of_type(Action)),
    ]

    # формирование файлов библиотеки
    latest_target_timestamp = get_newest_modification_time(args['target_dependencies'])
    for msg_type, idl_group in obj_list:
        if not idl_group:
            continue
        template_file = msg_type + '_lib.c.em'
        out_name = msg_type + '_lib.c'
        package_name = args['package_name']
        data = {
            'package_name': args['package_name'],
            'content': idl_group,
        }
        generated_file = os.path.join(
            args['output_dir'], msg_type, out_name)
        template = os.path.join(template_dir, template_file)
        # развертывание шаблона
        expand_template(
            template, data, generated_file,
            minimum_timestamp=latest_target_timestamp)
        generated_files.append(generated_file)

    return generated_files

# вспомогательная функция для формирования имен переменных
def make_prefix(tp):
    return '__'.join(tp.structure.namespaced_type.namespaces + [
                  convert_camel_case_to_lower_case_underscore(tp.structure.namespaced_type.name)])

# вспомогательная функция для заголовочных файлов
def make_include_prefix(tp):
    lst = tp.namespaced_type.namespaced_name()
    return '/'.join(
        [lst[0], lst[1], 'detail', convert_camel_case_to_lower_case_underscore(lst[2])])
```

### Папка bin

В данной папке находится Python скрипт, который вызывает написанную выше функцию, передавая ей аргументы командной строки: путь к файлу с агрументами генерации и список типов сообщений.

### Папка cmake

Для того чтобы генерация была запущена на этапе сборки ROS2 окружения, нужно настроить CMake. Основную работу выполняет скрипт *rosidl_generator_lua_generate_interfaces.cmake*.
```
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# локальные переменные
set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_lua/${PROJECT_NAME}")
set(_generated_c_files "")

# список имен файлов
foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_idl_name}" _module_name)
  set(_src_c "${_output_path}/${_parent_folder}/${_module_name}.c")
  list(APPEND _generated_c_files ${_src_c})

  # msg / srv / action в разные списки
  if(${_parent_folder} STREQUAL "msg")
    list(APPEND _msg_list ${_src_c})
  endif()
endforeach()

# добавляем файлы для формирования библиотеки (при необходимости)
if(NOT _msg_list STREQUAL "")
  list(INSERT _msg_list 0 "${_output_path}/msg/msg_lib.c")
  list(APPEND _generated_c_files "${_output_path}/msg/msg_lib.c")
endif()

file(MAKE_DIRECTORY "${_output_path}")

# поиск зависимостей
set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

# проверка наличия нужных файлов
set(target_dependencies
  "${rosidl_generator_lua_BIN}"
  ${rosidl_generator_lua_GENERATOR_FILES}
  "${rosidl_generator_lua_TEMPLATE_DIR}/idl.c.em"
  "${rosidl_generator_lua_TEMPLATE_DIR}/msg.c.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

# формирование файла настроек генератора
set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_lua__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_lua_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

set(_target_suffix "__lua")
set_property(
  SOURCE ${_generated_c_files}
  PROPERTY GENERATED 1
)

# команда вызова функции скрипта из папки *bin*
add_custom_command(
  OUTPUT ${_generated_c_files}
  COMMAND Python3::Interpreter
  ARGS ${rosidl_generator_lua_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  --typesupport-impls "${_typesupport_impls}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating Lua code for ROS interfaces"
  VERBATIM
)

# генерация C файлов
set(rosidl_generator_lua_suffix "__rosidl_generator_lua")
set(_target_name_lib "${rosidl_generate_interfaces_TARGET}${rosidl_generator_lua_suffix}")

add_library(${_target_name_lib} SHARED ${_generated_c_files})
target_link_libraries(${_target_name_lib}
  ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c)
add_dependencies(
  ${_target_name_lib}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__rosidl_typesupport_c
)
rosidl_get_typesupport_target(c_typesupport_target "${rosidl_generate_interfaces_TARGET}" "rosidl_typesupport_c")

# сборка сообщений
if(NOT _msg_list STREQUAL "")
  add_library(msg SHARED ${_msg_list})
  set_target_properties(msg PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${_output_path}
  )
  target_link_libraries(msg ${c_typesupport_target})
  ament_target_dependencies(msg "rosidl_runtime_c")
endif()

# аналогично для сервисов и экшенов
```

### Сборка

Осталось сделать ещё несколько шагов, чтобы выполнить сборку. Во первых, нужно создать в корне файл *rosidl_generator_lua-extras.cmake.in* с текстом
```
include("${CMAKE_CURRENT_LIST_DIR}/register_lua.cmake")
rosidl_generator_lua_extras(
  "${rosidl_generator_lua_DIR}/../../../lib/rosidl_generator_lua/rosidl_generator_lua"
  "${rosidl_generator_lua_DIR}/../../../@PYTHON_INSTALL_DIR@/rosidl_generator_lua/__init__.py"
  "${rosidl_generator_lua_DIR}/../resource"
)
```
Здесь прописаны пути к файлам и папкам, используемым в процессе генерации. Во-вторых, в *package.xml* добавляем следующие зависимости:
```xml
  <buildtool_depend>ament_cmake_export_assemblies</buildtool_depend>

  <buildtool_export_depend>ament_cmake</buildtool_export_depend>
  <buildtool_export_depend>rosidl_cmake</buildtool_export_depend>
  <buildtool_export_depend>rosidl_generator_c</buildtool_export_depend>
  <buildtool_export_depend>rosidl_typesupport_c</buildtool_export_depend>
  <buildtool_export_depend>rosidl_typesupport_interface</buildtool_export_depend>

  <build_depend>rosidl_runtime_c</build_depend>

  <exec_depend>rmw_implementation</exec_depend>
  <exec_depend>rmw_implementation_cmake</exec_depend>
  <exec_depend>rosidl_runtime_c</exec_depend>
  <exec_depend>rosidl_generator_c</exec_depend>
  <exec_depend>rosidl_parser</exec_depend>

  <member_of_group>rosidl_generator_packages</member_of_group>
```
Наконец, настраиваем сборку через *CMakeLists.txt*.
```
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rmw)

ament_index_register_resource("rosidl_generator_packages")
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS bin/rosidl_generator_lua
  DESTINATION lib/rosidl_generator_lua
)

install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)

# добавляем свои сценарии сборки
ament_package(
  CONFIG_EXTRAS
    "cmake/rosidl_generator_lua_get_typesupports.cmake"
    "cmake/register_lua.cmake"
    "rosidl_generator_lua-extras.cmake.in"
)
```

## Стандартные сообщения

Представленный выше код прекрасно справляется с генерацией сообщений, но есть нюанс. Он ориентирован на работу с кастомными сообщениями, т.е. описание которых лежит в вашем локальном рабочем окружении. Но ROS включает в себя множество стандартных типов, таких как std_msgs, nav_msgs, sensor_msgs и т.д. И нужно как-то обеспечить возможность работы с ними.

Можно попытаться собрать нужные стандартные библиотеки в своём локальном рабочем окружении. Однако в этом случае возникнет конфликт с глобальным окружением и ROS завершит работу. Я решил проблему следующим образом. Описания стандартных сообщений, т.е. msg и idl файлы, лежат в папке *ros_distro_name/share*. Я добавил отдельный пакет, который считывает эти описания, генерирует нужные динамические библиотеки и добавляет пути в *LUA_CPATH*. Это позволило иметь локальные версии библиотек не конфликтуя с глобальным окружением. Правда, пришлось использовать непубличные функции из *ament_cmake*, поэтому при изменении в них код придётся дорабатывать.

## Заключение
