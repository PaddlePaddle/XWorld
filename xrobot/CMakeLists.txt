cmake_minimum_required(VERSION 3.0)

project(xrobot CXX C)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
## -Wno-error for gcc 5.x
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -DBT_USE_DOUBLE_PRECISION=1 -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-error")

# SSE2
if((CMAKE_CXX_COMPILER_ID MATCHES "GNU") OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  add_compile_options(-msse2)
  message(STATUS "SSE2 instruction set")
elseif((CMAKE_CXX_COMPILER_ID MATCHES "Intel"))
  add_compile_options(/QxSSE2)
  message(STATUS "Intel SSE2 instruction set")
endif()

# SSE3
if((CMAKE_CXX_COMPILER_ID MATCHES "GNU") OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  add_compile_options(-msse3)
  message(STATUS "SSE3 instruction set")
elseif((CMAKE_CXX_COMPILER_ID MATCHES "Intel"))
  add_compile_options(/QxSSE3)
  message(STATUS "Intel SSE3 instruction set")
endif()

# AVX
if((CMAKE_CXX_COMPILER_ID MATCHES "GNU") OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  add_compile_options(-mavx)
  message(STATUS "AVX instruction set")
elseif((CMAKE_CXX_COMPILER_ID MATCHES "Intel"))
  add_compile_options(/QxAVX)
  message(STATUS "Intel AVX instruction set")
endif()

# AVX2
if((CMAKE_CXX_COMPILER_ID MATCHES "GNU") OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  add_compile_options(-mavx2)
  message(STATUS "AVX2 instruction set")
elseif((CMAKE_CXX_COMPILER_ID MATCHES "Intel"))
  add_compile_options(/QxAVX2)
  message(STATUS "Intel AVX2 instruction set")
endif()


include(ExternalProject)

set(EXTERNAL_PROJECT_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/external")


find_package(Threads REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python system)

## user can directly set GLOG_INCLUDE_DIRS and GLOG_LIBRARIES
if(NOT GLOG_LIBRARIES)
  # user can set GLOG_ROOT_DIR to help find_package
  find_package(Glog REQUIRED)
endif()
## user can directly set GFLAGS_INCLUDE_DIRS and GFLAGS_LIBRARIES
if(NOT GFLAGS_LIBRARIES)
  # user can set GFLAGS_ROOT_DIR to help find_package
  find_package(Gflags REQUIRED)
endif()

include(bullet)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIR})

set(DEP_LIBS
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES})

set(DEP_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS})

add_subdirectory(render_engine)

set(XROBOT_SOURCES
    vendor/json.cpp
    map_grid.cpp
    map_suncg.cpp
    state_machine.cpp
    task_example.cpp
    room_generator.cpp
    utils.cpp
    lidar.cpp
    navigation.cpp
    inventory.cpp
    world.cpp
    python_binding.cpp
)

add_library(xrobot SHARED
    ${XROBOT_SOURCES}
    $<TARGET_OBJECTS:render_engine>)

add_dependencies(xrobot render_engine bullet)

target_link_libraries(xrobot PUBLIC ${DEP_LIBS})

target_include_directories(xrobot PUBLIC ${DEP_INCLUDES})

set_target_properties(xrobot
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(xrobot_test test.cpp)
# add_executable(xrobot_playround test_playground.cpp)

target_link_libraries(xrobot_test PUBLIC xrobot ${DEP_LIBS})
# target_link_libraries(xrobot_playround PUBLIC xrobot ${DEP_LIBS})

target_include_directories(xrobot_test PUBLIC ${DEP_INCLUDES})
# target_include_directories(xrobot_playround PUBLIC ${DEP_INCLUDES})

set_target_properties(xrobot_test
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
# set_target_properties(xrobot_playround
#     PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})



