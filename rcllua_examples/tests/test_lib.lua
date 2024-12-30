-- rcllua_unit example
-- Feel free to use any test library you prefer

local rut = require 'rcllua_unit.testing'

local lib

-- Define tests as memeters of rut library
function rut:first()
  -- call inside function to catch load error
  lib = require 'rcllua_examples.library'

  -- check with asserts
  assert(lib.sum(1, 2) == 3)

  -- check with additional comments in the case of error
  assert( rut:eql(lib.sum(1, 2), 3) )
end

function rut:second()
  -- check float equality (tol = 0.001)
  assert( rut:eqlf(lib.sum(1.0, 2.0), 3.0) )

  -- check with specific tolerance
  assert( rut:eqlf(math.pi, 355.0/113, 1E-4) )
end

function rut:third()
  -- when expected error
  rut:catch(lib.raise)
end

-- execute tests
rut:run()

-- call in terminal 
--   colcon test --packages-select rcllua_examples
-- to see the result, add option
--   colcon test --packages-select rcllua_examples --event-handlers console_direct+

