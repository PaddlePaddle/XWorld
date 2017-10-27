add_library(py_simulator
  SHARED
  py_simulator.cpp)
set_target_properties(py_simulator PROPERTIES PREFIX "")  ## remove 'lib' prefix

link_simulator_exe(py_simulator) ## link libsimulator to py_simulator

set_target_properties(py_simulator
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
