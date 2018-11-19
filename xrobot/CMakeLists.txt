cmake_minimum_required(VERSION 3.0)

project(xrobot CXX C)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

option(CPP_EXAMPLES_ENABLE "c++ examples" ON)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
## -Wno-error for gcc 5.x
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -DBT_USE_DOUBLE_PRECISION=1 -std=c++11 -fPIC -Wno-deprecated-declarations -Wno-error")

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
    room_generator.cpp
    utils.cpp
    lidar.cpp
    navigation.cpp
    inventory.cpp
    world.cpp
    python_binding.cpp
)

set(XROBOT_CPP_EXAMPLE_SOURCES
    cpp_example/task_example_navigation_in_maze.cpp
    cpp_example/task_example_follow_robot.cpp
    cpp_example/task_example_suncg.cpp
    cpp_example/task_example_crowd.cpp
    cpp_example/task_example_obj.cpp
)

if(CPP_EXAMPLES_ENABLE)
    add_library(xrobot SHARED
        ${XROBOT_CPP_EXAMPLE_SOURCES}
        ${XROBOT_SOURCES}
        $<TARGET_OBJECTS:render_engine>
    )
else()
    add_library(xrobot SHARED
        ${XROBOT_SOURCES}
        $<TARGET_OBJECTS:render_engine>
    )
endif()

add_dependencies(xrobot render_engine bullet)

target_link_libraries(xrobot PUBLIC ${DEP_LIBS})

target_include_directories(xrobot PUBLIC ${DEP_INCLUDES})

set_target_properties(xrobot
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(CPP_EXAMPLES_ENABLE)
    # Navigation in Maze
    add_executable(xrobot_test_nav cpp_example/test_navigation_in_maze.cpp)

    target_link_libraries(xrobot_test_nav PUBLIC xrobot ${DEP_LIBS})

    target_include_directories(xrobot_test_nav PUBLIC ${DEP_INCLUDES})

    set_target_properties(xrobot_test_nav
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cpp_example)


    # Follow Robot
    add_executable(xrobot_test_follow_robot cpp_example/test_follow_robot.cpp)

    target_link_libraries(xrobot_test_follow_robot PUBLIC xrobot ${DEP_LIBS})

    target_include_directories(xrobot_test_follow_robot PUBLIC ${DEP_INCLUDES})

    set_target_properties(xrobot_test_follow_robot
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cpp_example)

    # SUNCG
    add_executable(xrobot_test_suncg cpp_example/test_suncg.cpp)

    target_link_libraries(xrobot_test_suncg PUBLIC xrobot ${DEP_LIBS})

    target_include_directories(xrobot_test_suncg PUBLIC ${DEP_INCLUDES})

    set_target_properties(xrobot_test_suncg
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cpp_example)

    # Crowd
    add_executable(xrobot_test_crowd cpp_example/test_crowd.cpp)

    target_link_libraries(xrobot_test_crowd PUBLIC xrobot ${DEP_LIBS})

    target_include_directories(xrobot_test_crowd PUBLIC ${DEP_INCLUDES})

    set_target_properties(xrobot_test_crowd
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cpp_example)

    # OBJ Scene
    add_executable(xrobot_test_obj cpp_example/test_obj.cpp)

    target_link_libraries(xrobot_test_obj PUBLIC xrobot ${DEP_LIBS})

    target_include_directories(xrobot_test_obj PUBLIC ${DEP_INCLUDES})

    set_target_properties(xrobot_test_obj
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cpp_example)
endif()


