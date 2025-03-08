-- Copyright 2025 Stanislav Mikhel
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--     http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- Generate instructions based on doxigen (C) and luadoc (Lua) used in project.
-- This parser is too simple for working with complex expressions and it is
-- expected documentation lines of specific form.
-- In C it should include elements 'Table', 'Method' and optional 'Arguments' and 'Return'.
-- In Lua only @param and @return can be recognized.

local C_FILES = {
  "../src/context.c",
  "../src/node.c",
  "../src/publisher.c",
  "../src/subscriber.c",
  "../src/client.c",
  "../src/service.c",
  "../src/clock.c",
  "../src/time.c",
  "../src/timer.c",
  "../src/logger.c",
  "../src/qos.c",
  "../src/utils.c",
  "../src/wait_set.c",
  "../src/action_client.c",
  "../src/action_server.c",
  "../src/lifecycle.c",
  "../src/guard_condition.c",
}

local LUA_FILES = {
  "../rcllua/rcllua.lua",
  "../rcllua/Node.lua",
  "../rcllua/Executor.lua",
  "../rcllua/client.lua",
  "../rcllua/Parameter.lua",
  "../rcllua/LifecycleNode.lua",
  "../rcllua/ActionClient.lua",
  "../rcllua/ActionServer.lua",
}

--- Get autodoc lines.
--  @param fname File name.
--  @param beg Comments that open block of documentation.
--  @param cont Comments in the begining of each next text line.
--  @param nextline When true, read line after the last comment.
--  @return list of autodoc blocks.
local function find_blocks (fname, beg, cont, nextline)
  local acc, block = {}, nil
  for s in io.lines(fname) do
    -- assume no block after block and no block in the end of file
    if string.find(s, beg) then
      block = {s}
    elseif block then
      if s:find(cont) then
        block[#block+1] = s
      else
        if nextline then block[#block+1] = s end
        acc[#acc+1] = block
        block = nil
      end
    end
  end
  return acc
end

--- Remove whitespaces in the begin and end of line.
--  @param s String for processing.
--  @return stripped line.
local function strip(s) return s:match("^%s*(.-)%s*$") end

-- Indices of documentation strings after parsing.
local DESCRIPTION, ARGS, RESULT = 1, 2, 3

--- Find components of C text block.
--  @param block List of strings.
--  @param acc Table for saving result.
local function parse_c (block, acc)
  if #block <= 1 then
    return  -- ignore short
  end
  -- parse
  local txt, ind, tbl = {{}, {}, {}}, DESCRIPTION, nil
  for _, s in ipairs(block) do
    s = s:match("^[%s*/]*(.*)")
    if #s > 1 and string.sub(s, 1, 1) ~= '\\' then
      if s:find("^Arguments") then
        ind = ARGS
      elseif s:find("^Return") then
        ind = RESULT
      elseif s:find("^Method") then
        txt.method = strip(s:sub(8))
      elseif string.find(s, "^Table") then
        tbl = strip(s:sub(7))
      else
        table.insert(txt[ind], s)
      end
    end
  end
  -- generate arguments
  local args = {}
  for i, v in ipairs(txt[ARGS]) do
    local a, b = v:sub(1, 2), v:sub(3)
    if a == '- ' then
      txt[ARGS][i] = string.format('%sarg%d %s', a, i, b)
      args[#args+1] = 'arg'..tostring(i)
    end
  end
  -- save
  if tbl and txt.method then
    if not acc[tbl] then acc[tbl] = {} end
    txt.method = string.format('%s (%s)', txt.method, table.concat(args, ', '))
    table.insert(acc[tbl], txt)
  end
end

--- Find components of Lua text block.
--  @param block List of strings.
--  @param acc Table for saving result.
local function parse_lua (block, acc)
  if #block <= 1 then
    return  -- ignore short
  end
  local txt, ind, tbl = {{}, {}, {}}, DESCRIPTION, nil
  for i, s in ipairs(block) do
    s = s:match("^[%s-]*(.*)")
    if i == #block then
      local a, b = s:find('function')
      if a and not s:find('local') then
        local name = nil
        if a == 1 then
          name = s:sub(b+1)
        else
          a = s:find('=')  -- expected "... = function ..."
          name = s:sub(1, a-1) .. s:sub(b+1)
        end
        a = name:find('%.')
        if a then
          tbl = strip(name:sub(1, a-1))
          txt.method = strip(name:sub(a+1))
        end
        -- don't show metamethods
        if txt.method and txt.method:find('^__') then
          return
        end
      end
    elseif s:find('^@param') then
      table.insert(txt[ARGS], '- '..s:sub(8))
    elseif s:find('^@return') then
      table.insert(txt[RESULT], '- '..s:sub(9))
    else
      table.insert(txt[DESCRIPTION], s)
    end
  end
  -- save
  if tbl and txt.method then
    if not acc[tbl] then acc[tbl] = {} end
    table.insert(acc[tbl], txt)
  end
end

--- Read and parse C file, save result to table.
--  @param fname C file name.
--  @param acc Table to store the result.
local function read_c_file (fname, acc)
  local txt = find_blocks(fname, '^/%*%*', '^ %*', false)
  for i = 1, #txt do
    parse_c(txt[i], acc)
  end
end

--- Read and parse Lua file, save result to table.
--  @param fname Lua file name.
--  @param acc Table to store the result.
local function read_lua_file (fname, acc)
  local txt = find_blocks(fname, '^%-%-%-', '^%-%-', true)
  for i = 1, #txt do
    parse_lua(txt[i], acc)
  end
end

--- Save found code documentation to file.
--  @param f File object.
--  @param title Section title.
--  @param acc Table with documentation.
local function write_to_file (f, title, acc)
  f:write('## ', title, '\n\n')
  for nm, grp in pairs(acc) do
    f:write('### ', nm, '\n')
    table.sort(grp, function (a, b) return a.method < b.method end)
    for _, t in ipairs(grp) do
      f:write('\n**', t.method, '**\n\n')
      for _, v in ipairs(t[DESCRIPTION]) do f:write(v, '\n') end
      for _, v in ipairs(t[ARGS]) do
        local p, q = v:match('^- ([%w_]+)%s+(.+)')
        if p then
          f:write(string.format('- *%s* %s\n', p, q))
        else
          f:write(v, '\n')
        end
      end
      if #t[RESULT] > 0 then
        f:write('\nReturns\n')
        for _, v in ipairs(t[RESULT]) do f:write(v, '\n') end
      end
    end
    f:write('\n')
  end
end

--- Generate markdown file with documentation.
--  @param c_arr List of C files.
--  @param lua_arr list of Lua files.
local function generate_for (c_arr, lua_arr)
  -- parse description in C files
  local acc_c = {}
  for i = 1, #c_arr do
    read_c_file(c_arr[i], acc_c)
  end
  -- parse description in Lua files
  local acc_lua = {}
  for i = 1, #lua_arr do
    read_lua_file(lua_arr[i], acc_lua)
  end

  -- save
  local name = "funcitons.md"
  local f = io.open(name, 'w')
  f:write('# RCLLUA methods\n\n')
  f:write('This page is generated based on files in *src* and *rcllua* directories.\n\n')
  write_to_file(f, "C part", acc_c)
  write_to_file(f, "Lua part", acc_lua)
  f:close()
  print("Save file:", name)
end

-- execute
generate_for (C_FILES, LUA_FILES)

