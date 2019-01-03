add_library(game_engine OBJECT 
  map_grid.cpp
  map_suncg.cpp
  state_machine.cpp
  room_generator.cpp
  lidar.cpp
  navigation.cpp
  inventory.cpp
  world.cpp
  robot.cpp
  python_binding.cpp)

add_dependencies(game_engine render_engine bullet_engine)

target_include_directories(game_engine PUBLIC
  ../
  ${DEP_INCLUDES})
