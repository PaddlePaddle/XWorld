if (MALMO_ROOT)
  message("Info: minecraft is added.")
  set(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/export_vars.sh MALMO_ROOT ${MALMO_ROOT}")
  execute_process(COMMAND bash "-c" ${COMMAND})

  add_definitions(-DPY_MALMO)
  add_subdirectory(games/minecraft)
  set(OBJECT_LIBS_LIST ${OBJECT_LIBS_LIST} $<TARGET_OBJECTS:minecraft>)
  get_directory_property(MINECRAFT_DEP_LIBS
    DIRECTORY games/minecraft
    DEFINITION DEP_LIBS)
  set(DEP_LIBS ${DEP_LIBS} ${MINECRAFT_DEP_LIBS})
  if(NOT OpenCV_DIR)
    add_dependencies(minecraft opencv)
  endif()
else()
  message("Warning: minecraft is not added.")
endif()

if (WITH_DEEPMIND_LAB)
  message("Info: Deepmind Lab is added.")

  add_definitions(-DPY_DEEPMIND_LAB)
  include(deepmind_lab)
  add_subdirectory(games/deepmind_lab)
  set(OBJECT_LIBS_LIST ${OBJECT_LIBS_LIST} $<TARGET_OBJECTS:dmlab>)
  if(NOT OpenCV_DIR)
    add_dependencies(dmlab opencv)
  endif()
else()
  message("Warning: Deepmind Lab is not added.")
endif()

if (WITH_ROBO_SCHOOL)
  message("Info: RoboSchool is added.")

  add_definitions(-DPY_ROBO_SCHOOL)
  add_subdirectory(games/roboschool)
  set(OBJECT_LIBS_LIST ${OBJECT_LIBS_LIST} $<TARGET_OBJECTS:roboschool>)
  get_directory_property(ROBO_SCHOOL_DEP_LIBS
    DIRECTORY games/roboschool
    DEFINITION DEP_LIBS)
  set(DEP_LIBS ${DEP_LIBS} ${ROBO_SCHOOL_DEP_LIBS})
  if(NOT OpenCV_DIR)
    add_dependencies(roboschool opencv)
  endif()
else()
  message("Warning: RoboSchool is not added.")
endif()
