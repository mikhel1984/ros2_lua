
local DESCRIPTION, ARGS, RESULT = 1, 2, 3
local function find_blocks (fname, beg, cont, nextline)
  local acc, block = {}, nil
  for s in io.lines(fname) do
    -- assume no block after block and no block in the end of file
    if string.find(s, beg) then
      block = {s}
    elseif block then
      if string.find(s, cont) then
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

local function strip(s) return string.match(s, "^%s*(.-)%s*$") end

local function parse_c (block, acc)
  if #block <= 1 then
    return  -- ignore short
  end
  -- parse 
  local txt, ind, tbl = {{}, {}, {}}, DESCRIPTION, nil
  for _, s in ipairs(block) do
    s = string.match(s, "^[%s*/]*(.*)")
    if #s > 1 and string.sub(s, 1, 1) ~= '\\' then 
      if string.find(s, "^Arguments") then
        ind = ARGS
      elseif string.find(s, "^Return") then
        ind = RESULT
      elseif string.find(s, "^Method") then
        txt.method = strip(string.sub(s, 8))
      elseif string.find(s, "^Table") then
        tbl = strip(string.sub(s, 7))
      else
        table.insert(txt[ind], s)
      end
    end
  end
  local args = {}
  for i, v in ipairs(txt[ARGS]) do
    local a, b = string.sub(v, 1, 2), string.sub(v, 3)
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

local function parse_lua (block, acc)
  if #block <= 1 then
    return  -- ignore short
  end
  local txt, ind, tbl = {{}, {}, {}}, DESCRIPTION, nil
  for i, s in ipairs(block) do
    s = string.match(s, "^[%s-]*(.*)")
    if i == #block then
      local a, b = string.find(s, 'function')
      if a and not string.find(s, 'local') then
        local name = nil
        if a == 1 then
          name = string.sub(s, b+1)
        else
          a = string.find(s, '=')  -- expected "... = function ..."
          name = string.sub(s, 1, a-1) .. string.sub(s, b+1)
        end
        a = string.find(name, '%.')
        if a then
          tbl = strip(string.sub(name, 1, a-1))
          txt.method = strip(string.sub(name, a+1))
        end
      end
    elseif string.find(s, '^@param') then
      table.insert(txt[ARGS], '- '..string.sub(s, 8))
    elseif string.find(s, '^@return') then
      table.insert(txt[RESULT], '- '..string.sub(s, 9))
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

local function read_c_file (fname)
  local txt = find_blocks(fname, '^/%*%*', '^ %*', false)
  local acc = {}
  for i = 1, #txt do
    local blk = txt[i]
    parse_c (blk, acc)
  end
  return acc
end

local function read_lua_file (fname)
  local txt = find_blocks(fname, '^%-%-%-', '^%-%-', true)
  local acc = {}
  for i = 1, #txt do
    local blk = txt[i]
    parse_lua(blk, acc)
  end
  return acc
end

local function sort_name (a, b) return a.method < b.method end

local function write_to_file (f, title, acc)
  f:write('## ', title, '\n\n')
  for nm, grp in pairs(acc) do
    f:write('### ', nm, '\n')
    table.sort(grp, sort_name)
    for _, t in ipairs(grp) do
      f:write('\n**', t.method, '**\n\n')
      for _, v in ipairs(t[DESCRIPTION]) do f:write(v, '\n') end
      for _, v in ipairs(t[ARGS]) do 
        local p, q = string.match(v, '^- ([%w_]+)%s+(.+)')
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
  end
  f:write('\n\n')
end

local acc = read_c_file("../src/node.c")
--local acc = read_lua_file("../rcllua/rcllua.lua")
local f = io.open('test.md', 'w')
write_to_file(f, 'The test', acc)
f:close()
