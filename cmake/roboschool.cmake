message("External project - roboschool")

find_package(PkgConfig)

function(append_libraries lib_list library_dir libraries)
    set(tmp_list "")
    foreach(lib ${libraries})
        list(APPEND tmp_list "${library_dir}/lib${lib}.so")
    endforeach(lib)
    set(${lib_list} ${tmp_list} PARENT_SCOPE)
endfunction(append_libraries)

## assimp
ExternalProject_Add(assimp
  PREFIX ${EXTERNAL_PROJECT_PREFIX}
  GIT_REPOSITORY "https://github.com/yu239/assimp"
  GIT_TAG "master"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/external/assimp"
  INSTALL_DIR "${EXTERNAL_PROJECT_PREFIX}/assimp"
  LOG_DOWNLOAD ON
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
)

ExternalProject_Get_Property(assimp INSTALL_DIR)
set(ASSIMP_INCLUDE_PATH "${INSTALL_DIR}/include")
set(ASSIMP_LIBRARIES "${INSTALL_DIR}/lib/libassimp.so.4")

# glm
ExternalProject_Add(glm
  PREFIX ${EXTERNAL_PROJECT_PREFIX}
  GIT_REPOSITORY "https://github.com/g-truc/glm.git"
  GIT_TAG "0.9.8.4"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/external/glm"
  INSTALL_DIR "${EXTERNAL_PROJECT_PREFIX}/glm"
  LOG_DOWNLOAD ON
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
)

ExternalProject_Get_Property(glm INSTALL_DIR)
set(GLM_INCLUDE_PATH "${INSTALL_DIR}/include")

## bullet physics
ExternalProject_Add(bullet
  PREFIX ${EXTERNAL_PROJECT_PREFIX}
  GIT_REPOSITORY "https://github.com/skylian/bullet3"
  GIT_TAG "master"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/external/bullet"
  INSTALL_DIR "${EXTERNAL_PROJECT_PREFIX}/bullet"
  LOG_DOWNLOAD ON
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DBUILD_SHARED_LIBS=ON
    -DUSE_DOUBLE_PRECISION=1
    -DBUILD_CPU_DEMOS=OFF
    -DBUILD_BULLET2_DEMOS=OFF
    -DBUILD_EXTRAS=OFF
    -DBUILD_UNIT_TESTS=OFF
    -DBUILD_CLSOCKET=OFF
    -DBUILD_ENET=OFF
    -DBUILD_OPENGL3_DEMOS=OFF
)

ExternalProject_Get_Property(bullet INSTALL_DIR)
set(BULLET_INCLUDE_PATHS "${INSTALL_DIR}/include" "${INSTALL_DIR}/include/bullet")
set(BULLET_LIBRARIES
  "${INSTALL_DIR}/lib/libLinearMath.so"
  "${INSTALL_DIR}/lib/libBullet3Common.so"
  "${INSTALL_DIR}/lib/libBulletCollision.so"
  "${INSTALL_DIR}/lib/libBulletDynamics.so"
  "${INSTALL_DIR}/lib/libBulletInverseDynamics.so"
  "${INSTALL_DIR}/lib/libPhysicsClientC_API.so")

## roboschool
ExternalProject_Add(roboschool
  PREFIX ${EXTERNAL_PROJECT_PREFIX}
  GIT_REPOSITORY "https://github.com/skylian/roboschool"
  GIT_TAG "remove_qt"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/external/roboschool"
  INSTALL_DIR "${EXTERNAL_PROJECT_PREFIX}/roboschool"
  DEPENDS assimp bullet glm
  LOG_DOWNLOAD ON
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  CONFIGURE_COMMAND ""
)

if (NOT OPENCV_FOUND)
  add_dependencies(roboschool opencv)
endif()

ExternalProject_Get_Property(roboschool SOURCE_DIR)
set(ROBOSCHOOL_DIR ${SOURCE_DIR})
## make roboschool
set(ROBOSCHOOL_INCLUDE_DEPS ${ASSIMP_INCLUDE_PATH} ${GLM_INCLUDE_PATH} ${BULLET_INCLUDE_PATHS} ${OpenCV_INCLUDE_DIRS})
set(ROBOSCHOOL_LIB_DEPS ${ASSIMP_LIBRARIES} ${BULLET_LIBRARIES} ${OPENCV_LIBS})

## custom make
add_custom_command(TARGET roboschool POST_BUILD
  COMMAND "python"
  ARGS make.py ${ROBOSCHOOL_INCLUDE_DEPS} ${ROBOSCHOOL_LIB_DEPS}
  WORKING_DIRECTORY "${ROBOSCHOOL_DIR}/roboschool/cpp-household"
)

set(ROBOSCHOOL_INCLUDE_PATH "${ROBOSCHOOL_DIR}/roboschool/cpp-household")
set(ROBOSCHOOL_LIBRARIES "${ROBOSCHOOL_DIR}/roboschool/libroboschool.so")

## export libs to the upper level
set(DEP_LIBS ${DEP_LIBS} ${ROBOSCHOOL_LIB_DEPS} ${ROBOSCHOOL_LIBRARIES})
