file(GLOB TASKS_SRC "./tasks/*.cpp")

add_library(xworld
  OBJECT
  xworld_simulator.cpp
  xworld_task.cpp
  ./xworld/xworld.cpp
  ./xworld/xmap.cpp
  ./xworld/xitem.cpp
  ./xworld/xagent.cpp
  ./xworld/xmaze.cpp
  ./xworld/xworld_parser.cpp
  ${TASKS_SRC}
  )
