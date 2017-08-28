-- Simple 3x3 DeepMind Lab map for testing.
-- Agent is in the middle with random pose (seed is fixed).
-- Agent is surrounded with 8 apples with +1 reward.
-- For details, please refer to https://github.com/deepmind/lab/blob/master/docs/lua_api.md

local make_map = require 'common.make_map'
local pickups = require 'common.pickups'

local api = {}

function api:start(episode, seed)
  -- fix random seed for testing.
  make_map.seedRng(1)
end

function api:commandLine(oldCommandLine)
  return make_map.commandLine(oldCommandLine)
end

function api:createPickup(className)
  return pickups.defaults[className]
end

function api:nextMap()
  return make_map.makeMap("demo_map", "AAA\nAPA\nAAA")
end

return api
