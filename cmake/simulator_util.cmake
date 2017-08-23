function(link_simulator_exe TARGET_NAME)
  target_link_libraries(${TARGET_NAME}
    simulator
    ${EXTERNAL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${CMAKE_DL_LIBS}
    )
endfunction()

function(link_simulator_test TARGET_NAME)
  link_simulator_exe(${TARGET_NAME})
  target_link_libraries(${TARGET_NAME}
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )
endfunction()
