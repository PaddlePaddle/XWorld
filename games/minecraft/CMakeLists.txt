set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${MALMO_ROOT}/cmake)
find_package(Boost COMPONENTS chrono date_time filesystem iostreams program_options regex system thread REQUIRED)
find_package(Xsd REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations") # get a lot of warnings from Boost about auto_ptr

add_library(minecraft
  OBJECT
  minecraft_simulator.cpp
  minecraft_simulator_demo.cpp
  )

set_source_files_properties(minecraft_simulator PROPERTIES COMPILE_FLAGS -Wno-error=non-virtual-dtor)
set_source_files_properties(minecraft_simulator_demo PROPERTIES COMPILE_FLAGS -Wno-error=non-virtual-dtor)

find_library(Malmo_LIBRARIES Malmo ${MALMO_ROOT}/build/install/Cpp_Examples/lib)
target_include_directories(minecraft
  PUBLIC
  ${MALMO_ROOT}/build/install/Cpp_Examples/include
  ${Boost_INCLUDE_DIR}
  ${XSD_INCLUDE_DIRS}
  )

set(DEP_LIBS
  ${Malmo_LIBRARIES}
  ${Boost_LIBRARIES}
  ${XSD_LIBRARIES}
  bz2)
