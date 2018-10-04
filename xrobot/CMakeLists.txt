cmake_minimum_required(VERSION 3.0)

project(xrobot CXX C)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
## -Wno-error for gcc 5.x
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DBT_USE_DOUBLE_PRECISION=1 -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-error")

include(ExternalProject)

set(EXTERNAL_PROJECT_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/external")

find_package(Threads REQUIRED)

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

set(DEP_LIBS
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES})

set(DEP_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS})

add_subdirectory(render_engine)

set(XROBOT_SOURCES
    vendor/json.cpp
    map.cpp
    map_suncg.cpp
    state_machine.cpp
    task_example.cpp
    room_generator.cpp
    utils.cpp
    lidar.cpp
    inventory.cpp
    world.cpp
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

target_link_libraries(xrobot_test PUBLIC xrobot ${DEP_LIBS})

target_include_directories(xrobot_test PUBLIC ${DEP_INCLUDES})

set_target_properties(xrobot_test
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})