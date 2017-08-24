add_library(dmlab OBJECT deepmind_lab_simulator.cpp dmlab_so_loader.cc)

add_dependencies(dmlab deepmind_lab)

include_directories(${DEEPMIND_LAB_INCLUDE_PATH})

target_compile_definitions(dmlab PRIVATE DMLAB_SO_LOCATION="libdmlab.so")
