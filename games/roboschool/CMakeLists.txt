set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(NumPy REQUIRED)

add_library(roboschool OBJECT robo_simulator.cpp)

if (PYTHON_NUMPY_FOUND)
    message("NumPy Found ${PYTHON_NUMPY_INCLUDE_DIR} with ${PYTHON_INCLUDE_DIRS}")
endif (PYTHON_NUMPY_FOUND)

target_include_directories(roboschool
    PUBLIC
    ${PYTHON_NUMPY_INCLUDE_DIR}
)

set_source_files_properties(robo_simulator PROPERTIES COMPILE_FLAGS -Wno-error=non-virtual-dtor)
