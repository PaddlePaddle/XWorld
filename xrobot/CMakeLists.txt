cmake_minimum_required(VERSION 3.0)

project(xrobot CXX C)

include(ExternalProject)
set(EXTERNAL_PROJECT_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/external")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

option(USE_DOUBLE_PRECISION "Use double precision floating point" ON)
option(BUILD_CPP_EXAMPLES "c++ examples" ON)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
## -Wno-error for gcc 5.x
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-error")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Threads REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python system)

#include_directories(${Boost_INCLUDE_DIRS})
#include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIR})

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

include(assimp)
include(glm)
include(glfw)
include(bullet)

set(DEP_LIBS
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES})

set(DEP_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS})

add_library(utils OBJECT
  vendor/json.cpp
  utils.cpp)

if (USE_DOUBLE_PRECISION)
  add_definitions(-DUSE_DOUBLE_PRECISION)
endif()

add_subdirectory(render_engine)

add_subdirectory(bullet_engine)

add_subdirectory(game_engine)

add_library(xrobot SHARED
  ${XROBOT_SOURCES}
  $<TARGET_OBJECTS:render_engine>
  $<TARGET_OBJECTS:bullet_engine>
  $<TARGET_OBJECTS:game_engine>
  $<TARGET_OBJECTS:utils>)

add_dependencies(xrobot utils render_engine bullet_engine game_engine)

target_link_libraries(xrobot PUBLIC ${DEP_LIBS})
target_include_directories(xrobot PUBLIC ${DEP_INCLUDES})

set_target_properties(xrobot
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(BUILD_CPP_EXAMPLES)
  # Navigation in Maze
  add_executable(xrobot_test_nav
    cpp_example/example_navigation_in_maze.cpp
    cpp_example/navigation_in_maze.cpp)
  add_dependencies(xrobot_test_nav xrobot)
  target_link_libraries(xrobot_test_nav PUBLIC xrobot ${DEP_LIBS})
  target_include_directories(xrobot_test_nav PUBLIC ${DEP_INCLUDES})
  set_target_properties(xrobot_test_nav
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/cpp_example)

  # Navigation in SUNCG
  add_executable(xrobot_test_suncg
    cpp_example/example_navigation_in_suncg.cpp
    cpp_example/navigation_in_suncg.cpp)
  add_dependencies(xrobot_test_nav xrobot)
  target_link_libraries(xrobot_test_suncg PUBLIC xrobot ${DEP_LIBS})
  target_include_directories(xrobot_test_suncg PUBLIC ${DEP_INCLUDES})
  set_target_properties(xrobot_test_suncg
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/cpp_example)
endif()


