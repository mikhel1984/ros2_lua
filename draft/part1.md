# Hello ROS2

Пару лет назад у меня возникла мысль, почему до сих пор нет полнофункционального порта ROS2 для языка программирования Lua? (Спойлер: потому что никому не нужно.) И я решил это исправить. Разработчики ROS2 вынесли базовый функционал в C библитеку [rcl](https://github.com/ros2/rcl), то есть достаточно создать обертку на каком-либо языке программирования, и можно пользоваться! На практике всё оказалось немного сложнее... В этой серии заметок я собираюсь описать те шаги, которые нужно выполнить, чтобы портировать ROS2 на свой любимый язык программирования.

## А нужно ли?

ROS2 содержит богатый набор [CLI](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html) команд. Если в вашем языке есть возможность форматирования строк и передачи команд операционной системе (функция типа **execute**()), этого может быть достаточно. Плюсы: быстро, просто, переносимо между версиями ROS2. Минусы: неэффективно, используется промежуточная интерпретация в виде Python, ограниченная функциональность. Этот способ подойдет для публикации данных, но организовать обработку входящих сообщений, особенно для нескольких топиков, может быть сложно или невозможно.

## Контекст

Прежде чем двигаться дальше, я бы хотел описать задачу, которую решал.

Разработка велась под ROS2 версии Humble. Хотя она уже устарела, большая часть информации будет актуальна и для более новых дистрибутивов.

Моей целью было создание библиотеки, которая органично вписывалась бы в ROS2 экосистему. То есть пользователь создает скрипт на Lua, но работает с ним, используя такие инструменты, как *colcon build*, *colcon test*, *ros2 run*, *ros2 launch* и т.д. Сам Lua скрипт должен иметь структуру, аналогичную коду на C++ или Python, чтобы не отпугивать потенциальных пользователей необычными конструкциями.

На случай применения во встраиваемых системах я постарался обойтись без сторонних библиотек Lua, достаточно иметь сам интерпретатор.

Для реализации данной цели потребовалось решить следующие задачи:
- создание библиотеки, реализующей основные сущности ROS2 (ноды, издатели, подписчики, сервисы, таймеры и пр.);
- создание Lua интерфейса для упрощения работы с фреймворком, а также реализация функционала, который не входит в rcl (спин, параметры);
- создание генераторов для трансляции сообщений;
- настройка системы сборки и хуков для работы с нодами из ROS2 окружения.

Далее я постараюсь абстрагироваться от Lua, насколько это возможно, используя его только в качестве иллюстративных примеров.

## Документация

Начать знакомство со структурой кода можно на [сайте](https://docs.ros.org/en/humble/Concepts/Advanced/About-Internal-Interfaces.html) ROS2. Однако наилучшая документация, которую мне удалось найти, это описание функций в самой библиотеке [rcl](https://github.com/ros2/rcl). Также полезно рассмотреть реализацию официальных клиентских библиотек [rclcpp](https://github.com/ros2/rclcpp), [rclpy](https://github.com/ros2/rclpy), [rclc](https://github.com/ros2/rclc), однако они перегружены дополнительным функционалом, разобраться в котором может быть непросто. Очень помогли сторонние библиотеки типа [ros2_dotnet](https://github.com/ros2-dotnet/ros2_dotnet), [ros2_rust](https://github.com/ros2-rust/ros2_rust), так как в них проще найти "скелет", который требуется реализовать.

## MVP

В качестве минимальной программы в ROS2 рассмотрим программу ?, которая по таймеру выводит в лог пользовательское сообщение. Затем постараемся этот код в систему ? colcon.

Типичный подход заключается в том, чтобы создать динамическую библиотеку (назовем её rclbind), которая содержит вызовы к низкоуровневым командам, следит за жизненным циклом объектов и т.п., а также собственно клиентскую библиотеку, упрощающую работу с ROS2 для конечного пользователя.

Реализуем обертку для основных элементов, требуемых для решения поставленной задачи.
Каждый объект *rcl* имеет конструктор по умолчанию (_get_zero_initialized), функцию инициализации (_init) и освобождения ресуросов (_fini). Задача клиентской библиотеки заключается, в том числе, в обеспечении правильного порядка их вызовов. Каждый вызов библиотечной функции возвращает код завершения, который должен быть обработан. Для сокращения записи эта часть будет опущена.

Состояние исполняемого ROS2 файла хранится в объекте *rcl_context_t*, который должен быть создан первым и последним освобожден. Библиотеки *rclcpp* и *rclpy* допускают наличие нескольких контекстов и создают их динамически. Если вы считаете, что одного контекста более чем достаточно, можно пойти по пути *rcldontnet* и создать глобальную статическую переменную.

В функции *rcl_lua_context_init* мы инициализируем инициализируем объект с учетом опций, полученных из аргументов командной строки. Можно расширить реализацию и считывать настройки из объекта *rcl_init_options_t*, который будет настраиваться до иницализации контекста. Также здесь мы инициализируем логирование, поскольку трудно представить пример практического использования, когда ROS2 нода не пользуется логами. Проверка статуса и освобождение ресурсов тривиальны и сводятся к вызову соответствующих функций.
```c
#include <rcl/allocator.h>
#include <rcl/init.h>
#include <rcl/init_options.h>
#include <rcl/logging.h>

static rcl_context_t context_;

// инициализация
// функция rclbind.context_init
static int rcl_lua_context_init (lua_State* L)
{
  // инициализация пустого объекта
  context_ = rcl_get_zero_initialized_context();

  // опции запуска по умолчанию
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);

  // заполнение полей контекста
  ret = rcl_init(argc, argv, &init_options, &context_);

  // инициализация логирования
  rcutils_ret_t rcret = rcutils_logging_initialize();

  return 0;
}

// проверка текущего состояния контекста
// функция rclbind.context_ok
static int rcl_lua_context_ok (lua_State* L)
{
  bool ok = rcl_context_is_valid(&context_);

  return 1;
}

// освобождение ресурсов
// функция rclbind.context_shutdown
static int rcl_lua_context_shutdown (lua_State* L)
{
  // контекст
  rcl_ret_t ret = rcl_shutdown(&context_);
  // логирование
  rcutils_ret_t rcret = rcutils_logging_shutdown();

  return 0;
}
```

Логирование мы инициализировали, теперь нужно добавить функцию, которая будет выводить сообщение пользователя. Для этого используем *rcutils_log*, первым аргументом тут является структура *rcutils_log_location_t*, содержащая дополнительную информацию о файле, функции и строке кода, но можно обойтись без нее.

```c
#include <rcutils/logging.h>

// упрощенная форма вывода сообщения в лог
// функция rclbind.simp_log
static int rcl_lua_logger_log_simp (lua_State* L)
{
  rcutils_log(NULL, severity, name, "%s", message);

  return 0;
}
```

Для того чтобы публиковать сообщения по таймеру, нам нужны два объекта: собственно таймер и часы, к которым он будет привязан (системные или реализации *ROS2*).
```c
#include <rcl/time.h>

// конструктор часов
// функция rclbind.new_clock
static int rcl_lua_clock_init (lua_State* L)
{
  rcl_clock_t* clock = lua_newuserdata(L, sizeof(rcl_clock_t));
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_clock_init(clock_type, clock, &allocator);

  return 1;
}

// деструктор часов
// вызывается сборщиком мусора
static int rcl_lua_clock_free (lua_State* L)
{
  rcl_ret_t ret = rcl_clock_fini(clock);

  return 0;
}
```

Для таймера на данный момент достаточно иметь возможность создания, освобождения, и запуска. Таймер срабатывает один раз, для многократного использования нужно повторно вызывать функцию запуска.
```c
#include <rcl/timer.h>

// конструктор таймера
// функция rclbind.new_timer
static int rcl_lua_timer_init (lua_State* L)
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_timer_t* timer = lua_newuserdata(L, sizeof(rcl_timer_t));
  *timer = rcl_get_zero_initialized_timer();

  rcl_ret_t ret = rcl_timer_init(
    timer, clock, context_, period_nsec, NULL, allocator);

  return 1;
}

// деструктор таймера
// вызывается сборщиком мусора
static int rcl_lua_timer_free (lua_State* L)
{
  rcl_ret_t ret = rcl_timer_fini(timer);

  return 0;
}

// запуск таймера
// метод timer.call
static int rcl_lua_timer_call (lua_State* L)
{
  rcl_ret_t ret = rcl_timer_call(timer);

  return 0;
}
```

Основной функционал ROS2 ноды обычно реализуется в коллбэках. За работу с ними отвечает объект *rcl_wait_set_t*, который запускается в цикле. На каждой итерации происходит заполнение данного объекта ссылками на такие объекты как подписчики, таймеры и прочие, с которыми могут быть связаны callback функции. Затем Wait Set переходит в блокирующий режим ожидания. Когда происходит ожидаемое событие, объект разблокируется и позволяет обработчику извлечь информацию об этом событии. Правда, для этого придется пройтись по всем сущностям, добавленным в Wait Set, и проверить, связано ли событие конкретно с ней.
```c
#include <rcl/wait.h>

// конструктор Wait Set
// функция rclbind.new_wait_set
static int rcl_lua_wait_set_init (lua_State* L)
{
  // инициализация для работы с заданным числом объектов
  rcl_wait_set_t* wait_set = lua_newuserdata(L, sizeof(rcl_wait_set_t));
  *wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t ret = rcl_wait_set_init(
    wait_set,
    (size_t) num_sub,     // число подписчиков
    (size_t) num_guard,   // число защитников
    (size_t) num_timers,  // число таймеров
    (size_t) num_cli,     // число клиентов
    (size_t) num_srv,     // число сервисов
    (size_t) num_ev,      // число событий
    context_,
    rcl_get_default_allocator());

  return 1;
}

// деструктор Wait Set
// вызывается сборщиком мусора
static int rcl_lua_wait_set_free (lua_State* L)
{
  rcl_ret_t ret = rcl_wait_set_fini(wait_set);

  return 0;
}

// очистка Wait Set, обычно вызывается в начале цикла
// метод waitset.clear
static int rcl_lua_wait_set_clear (lua_State* L)
{
  rcl_ret_t ret = rcl_wait_set_clear(wait_set);

  return 0;
}

// добавление таймера в Wait Set
// метод waitset.add_timer
static int rcl_lua_wait_set_add_timer (lua_State* L)
{
  size_t index = 0;
  rcl_ret_t ret = rcl_wait_set_add_timer(wait_set, timer, &index);

  return 1;
}

// блокирующее ожидание события
// метод waitset.wait
static int rcl_lua_wait_set_wait (lua_State* L)
{
  rcl_ret_t ret = rcl_wait(wait_set, timeout);
  switch (ret) {
    case RCL_RET_OK:
    case RCL_RET_TIMEOUT:
      // успешно
    default:
      // ошибка
  }

  return 1;
}

// проверка сработавших таймеров
// можно вернуть список таймеров или сразу добавить связанные с ними функции
// метод waitset.ready_timers
static int rcl_lua_wait_set_ready_timers (lua_State* L)
{
  for (size_t i = 0; i < wait_set->size_of_timers; i++) {
    if (wait_set->timers[i]))
    {
      // помещяем связанную функцию в список
    }
  }

  return 1;
}
```
Здесь показаны таймеры, однако для других объектов (подписчиков, клиентов и пр.) метод добавления в Wait Set и проверки состояния аналогичен.

После того как библиотека скомпилирована, можно написать программу. В случае Lua код может быть следующим.
```lua
-- подключение функций rcl
local rclbind = require("rcllua.rclbind")

-- callback для таймера
local function timer_cb ()
  rclbind.simp_log(rclbind.LogLevel.INFO, "rcllua", "Hello world")
end

-- инициализация контекста
rclbind.context_init(arg)

-- создание необходимых объектов
local clock = rclbind.new_clock(rclbind.ClockType.STEADY_TIME)
local timer = rclbind.new_timer(clock, 0.5, timer_cb)
local wait_set = rclbind.new_wait_set(0, 0, 1, 0, 0, 0)

while rclbind.context_ok() do
  -- запускаем таймер
  timer:call()
  -- добавляем таймер в список ожидания
  wait_set:clear()
  wait_set:add_timer(timer)
  -- ждем ...
  wait_set:wait(-1)

  -- здесь должно быть извлечение таймера и
  -- функции через wait_set:ready_timers()
  -- по поскольку других объектов в списке ожидания не было
  -- вызовем связанную функцию
  timer_cb()
end

-- освобождение контекста
rclbind.context_shutdown()
```
Здесь мы использовали "сырые" вызовы функций обертки над *rcl*, что, очевидно, неудобно для конечного пользователя, хотя и эффективно с точки зрения вычислений. Повышение уровня абстракции для удобства применения - задача следующего уровня клиентской библиотеки.

## adasf

Команда **ros2 run** выполняет поиск и запуск исполняемого файла, т.е. можно обойтись без нее, достаточно напрямую вызвать написанную программу.
Однако, если мы хотим полноценно пользоваться инструментами ROS2, нужно как минимум настроить сборку через **colcon build** и запуск через **ros2 run**.

В Lua нет стандартной системы сборки, во всяком случая я с такой не сталкивался. Поэтому использовал "стандартные" для ROS2 CMake и bash, и добавил в сборку пакет с cmake функциями.

Для того чтобы написанная нами программа была видна ROS-у, нужно указать, где её искать, а ещё желательно положить в нужное место и создать исполняемый файл. Во первых, создадим шаблон для обновления путей поиска исполняемых файлов. Здесь определяется функция для формирования списка путей без дубликатов и пустых строк, которая потом применяется к текущему пути. Данный файл на 95% процентов совпадает с автогенерируемым кодом в пакете ROS2, отличие заключается только в использовании разделителя ";", который принят в Lua. Если ваш язык программирования использует ":", такой файл можно не добавлять. 
```bash
ament_append_unique_value() {
  # аргументы
  _listname=$1
  _value=$2

  # проверка наличия переменной
  eval _values=\$$_listname
  _duplicate=
  _ament_append_unique_value_IFS=$IFS
  IFS=";"
  if [ "$AMENT_SHELL" = "zsh" ]; then
    ament_zsh_to_array _values
  fi
  for _item in $_values; do
    # пустые строки игнорируются
    if [ -z "$_item" ]; then
      continue
    fi
    if [ $_item = $_value ]; then
      _duplicate=1
    fi
  done
  unset _item

  # дубликаты игнорируются
  if [ -z "$_duplicate" ]; then
    # устранение начального разделителя
    if [ -z "$_values" ]; then
      eval $_listname=\"$_value\"
    else
      # конкатенация
      unset IFS
      eval $_listname=\"\$$_listname\;$_value\"
    fi
  fi
  IFS=$_ament_append_unique_value_IFS
  unset _ament_append_unique_value_IFS
  unset _duplicate
  unset _values
  unset _value
  unset _listname
}

if [ -z "$LUA_PATH" ]; then
  export LUA_PATH=";;$COLCON_CURRENT_PREFIX/lib/lua/?.lua"
else
  ament_append_unique_value LUA_PATH "$COLCON_CURRENT_PREFIX/lib/lua/?.lua"
fi
```

Чтобы шаблон был "подхвачен" при сборке, нужно создать переменную, имя которой оканчивается на _ENVIRONMENT_HOOK_REGISTERED. Для этого определим макрос в CMake, он переместит исполняемые файлы в директорию проекта install/имя_пакета/lib/lua, а также настроит обновление окружения при запуске install/setup.bash.
```
macro(rcllua_cmake_install_lib src_name)
  install(
    DIRECTORY ${src_name}
    DESTINATION lib/lua
  )

  if(NOT DEFINED _AMENT_CMAKE_LUA_ENVIRONMENT_HOOK_REGISTERED)
    set(_AMENT_CMAKE_LUA_ENVIRONMENT_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core QUIET REQUIRED)

    ament_environment_hooks(
      "${rcllua_cmake_DIR}/templates/env_hook_lua.sh.in")
  endif()
endmacro()
```

Для того чтобы срабатывала команда ros2 run необходим исполяемый файл. Можно обязать пользователя явно указывать shebang, но если он забудет это сделать, работоспособность нарушится. Я решил пойти тем же путем, что и rclpy, и генерировать новый файл, который при запуске вызывает пользовательскую программу. Для этого служит следующий макрос, создает файл с указанием пути к вызываемой программе, помещает его в install/имя_пакета/lib/имя_пакета и делает исполняемым.
```
set(RCLLUA_EXEC /usr/local/bin/lua)

macro(rcllua_cmake_executable src_name exec_name)
  set(_out_dir ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
  file(MAKE_DIRECTORY "${_out_dir}")
  # создаем файл
  file(WRITE "${_out_dir}/${exec_name}"
    "#!${RCLLUA_EXEC}\n"
    "local script = loadfile('${CMAKE_SOURCE_DIR}/${src_name}')\n"
    "script()"
  )
  # настраиваем права доступа
  file(CHMOD "${_out_dir}/${exec_name}"
    PERMISSIONS
      OWNER_READ OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
endmacro()
```

Теперь пользователь может добавить в CMakeLists.txt следующий код.
```
# зависимости
find_package(rcllua REQUIRED)
find_package(rcllua_cmake REQUIRED)

# создание исполяемого файла hello
rcllua_cmake_executable(${PROJECT_NAME}/hello.lua hello)
# для доступа из других пакетов и файлов
# можно настроить пути к текущему пакету 
rcllua_cmake_install_lib(${PROJECT_NAME})
```
После сборки настройки путей будут добавлены в install/setup.bash, а файлы будут доступны для вызова через ros2 run.

Код проекта [ros2_lua](https://github.com/mikhel1984/ros2_lua).

