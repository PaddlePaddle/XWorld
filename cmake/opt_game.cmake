if (WITH_ATARI)
  message("Info: Atari is added.")

  add_definitions(-DPY_ATARI)
  include(atari)
  add_subdirectory(games/arcade)
  set(OBJECT_LIBS_LIST ${OBJECT_LIBS_LIST} $<TARGET_OBJECTS:arcade>)
  if(NOT OpenCV_DIR)
    add_dependencies(arcade opencv)
  endif()
else()
  message("Warning: Atari is not added.")
endif()

if (ROBOSCHOOL_ROOT)
  message("Info: XWorld3D is added.")
 
  add_subdirectory(games/xworld3d)
  set(OBJECT_LIBS_LIST ${OBJECT_LIBS_LIST} $<TARGET_OBJECTS:xworld3d>)
  get_directory_property(XWORLD3D_DEP_LIBS
    DIRECTORY games/xworld3d
    DEFINITION DEP_LIBS)
  set(DEP_LIBS ${DEP_LIBS} ${XWORLD3D_DEP_LIBS})
  if(NOT OpenCV_DIR)
    add_dependencies(xworld3d opencv)
  endif()
else()
  message("Warning: XWorld3D is not added.")
endif()
