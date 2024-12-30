-- rcllua example
-- Code for sharing with other Lua scripts.

local lib = {}

function lib.sum (a, b)
  return a + b
end

function lib.raise ()
  error("Don't touch me!'")
end

return lib
