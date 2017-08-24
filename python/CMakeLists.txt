find_package(Boost REQUIRED python)
find_package(PythonLibs REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

add_library(py_simulator
  SHARED
  py_simulator.cpp)
set_target_properties(py_simulator PROPERTIES PREFIX "")  ## remove 'lib' prefix

link_simulator_exe(py_simulator) ## link libsimulator to py_simulator
target_link_libraries(py_simulator ${Boost_LIBRARIES}) ## Boost python needs to be linked

set_target_properties(py_simulator
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
