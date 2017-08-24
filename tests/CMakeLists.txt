add_executable(test_simple_game_simulator
  test_simple_game_simulator.cpp)
link_simulator_test(test_simple_game_simulator)
add_test(simulator_test_simple_game test_simple_game_simulator)

add_executable(test_simulator_seed
  test_simulator_seed.cpp)
link_simulator_test(test_simulator_seed)
add_test(simulator_test_simulator_seed test_simulator_seed)

add_executable(test_teacher_sentence_generator
  test_teacher_sentence_generator.cpp
  )
link_simulator_test(test_teacher_sentence_generator)
add_test(simulator_test_teacher_sentence_generator test_teacher_sentence_generator)

add_executable(test_xworld_parser
  test_xworld_parser.cpp
  )
link_simulator_test(test_xworld_parser)
add_test(simulator_test_xworld_parser test_xworld_parser)

add_executable(test_xworld_task
  test_xworld_task.cpp
  )
link_simulator_test(test_xworld_task)
add_test(simulator_test_xworld_task test_xworld_task)

if(WITH_DEEPMIND_LAB)
  add_executable(test_deepmind_lab_simulator
      test_deepmind_lab_simulator.cpp
  )
  link_simulator_test(test_deepmind_lab_simulator)
  add_test(simulator_test_deepmind_lab test_deepmind_lab_simulator)
endif()

if(WITH_ROBO_SCHOOL)
  find_package(PythonLibs REQUIRED)
  add_executable(test_robo_simulator
    test_robo_simulator.cpp
    )
  target_include_directories(test_robo_simulator
    PUBLIC
    ${PYTHON_INCLUDE_DIRS})
  link_simulator_test(test_robo_simulator)
  add_test(simulator_test_robo test_robo_simulator)
endif()
