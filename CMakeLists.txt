cmake_minimum_required(VERSION 3.0)
# Prerequisites: Boost, Glog, Gflags, Gtest
#
# Need to set the following when doing cmake:
# WITH_TESTING (optional, default ON)
# WITH_ATARI (optional, default OFF)
# WITH_XWORLD3D (optional, default OFF)
#
# 1. mkdir build; cd build
# 2. cmake -DWITH_TESTING=ON ..
# 3. make -j4

project(simulator CXX C)

option(WITH_ATARI "Include ATARI" OFF)
option(WITH_XWORLD3D "Include XWorld3D" OFF)
option(WITH_TESTING "Enable test cases" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
## -Wno-error for gcc 5.x
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-error")

include(ExternalProject)
include(simulator_util)

set(EXTERNAL_PROJECT_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/external")

find_package(Threads REQUIRED)
find_package(PythonLibs 2.7 EXACT REQUIRED)
find_package(Boost REQUIRED COMPONENTS python system)

set(DEP_LIBS ${Boost_LIBRARIES})

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

## user can set OPENCV_LIBS and OpenCV_INCLUDE_DIRS
if(NOT OPENCV_LIBS)
  set(OPENCV_FOUND FALSE)
  include(opencv)
else()
  set(OPENCV_FOUND TRUE)
endif()
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${GFLAGS_INCLUDE_DIRS})
include_directories(${GLOG_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SIMULATOR_SOURCES
  data_packet.cpp
  teacher.cpp
  teaching_task.cpp
  simulator.cpp
  simulator_util.cpp
  python/py_init.cpp
  simulator_communication.cpp
  simulator_interface.cpp
  )

set(EXTERNAL_LIBS
  ${PYTHON_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OPENCV_LIBS})

## all subdirectories produce OBJECT libraries
add_subdirectory(games/simple_game)
add_subdirectory(games/simple_race)
add_subdirectory(games/xworld)

# if we need to compile OpenCV, we need to set the dependencies
if(NOT OPENCV_FOUND)
  add_dependencies(simple_race opencv)
  add_dependencies(xworld opencv)
endif()

set(OBJECT_LIBS_LIST
  $<TARGET_OBJECTS:simple_game>
  $<TARGET_OBJECTS:simple_race>
  $<TARGET_OBJECTS:xworld>
  )

## include optional games
## this might increment OBJECT_LIBS_LIST, EXTERNAL_LIBS, and DEP_LIBS
## according to the games' needs
include(opt_game)

## put all simulators into one package
add_library(simulator SHARED
  ${SIMULATOR_SOURCES}
  ${OBJECT_LIBS_LIST})

if(WITH_TESTING)
  ## user can directly set GTEST_INCLUDE_DIRS and GTEST_BOTH_LIBRARIES
  if(NOT GTEST_LIBRARIES)
    # user can set GTEST_ROOT to help find_package
    find_package(GTest REQUIRED)
  endif()
  include_directories(${GTEST_INCLUDE_DIRS})
  enable_testing()
  add_subdirectory(tests)
else()
  message("Warning: GTEST turned off")
endif()

add_subdirectory(python)
add_subdirectory(examples)

target_link_libraries(simulator PUBLIC
  ${DEP_LIBS}
  )

set_target_properties(simulator
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
