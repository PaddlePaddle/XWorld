function(add_test_executable TARGET_NAME)
  add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
  link_simulator_exe(${TARGET_NAME})
  set_target_properties(${TARGET_NAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

add_test_executable(test_atari)
add_test_executable(test_simple_race)
add_test_executable(test_xworld)
