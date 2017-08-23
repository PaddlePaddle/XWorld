set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(PkgConfig REQUIRED)
pkg_search_module(PYTHON REQUIRED python)
find_package(Boost REQUIRED python)
find_package(NumPy REQUIRED)

add_library(roboschool OBJECT robo_simulator.cpp)

if (PYTHON_NUMPY_FOUND)
    message("NumPy Found ${PYTHON_NUMPY_INCLUDE_DIR} with ${PYTHON_INCLUDE_DIRS}")
endif (PYTHON_NUMPY_FOUND)

target_include_directories(roboschool
    PUBLIC
    ${PYTHON_NUMPY_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
)

set(DEP_LIBS
    ${DEP_LIBS}
    ${PYTHON_LIBRARIES}
    ${Boost_LIBRARIES})

set_source_files_properties(robo_simulator PROPERTIES COMPILE_FLAGS -Wno-error=non-virtual-dtor)
