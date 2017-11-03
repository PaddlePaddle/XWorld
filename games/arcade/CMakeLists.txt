include(atari)

add_library(arcade OBJECT arcade_simulator.cpp)

add_dependencies(arcade atari)

target_include_directories(arcade
  PUBLIC
  ${ATARI_INCLUDE_PATH})
