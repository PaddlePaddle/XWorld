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
  GIT_REPOSITORY "https://github.com/yu239/assimp"
  GIT_TAG "master"
  LOG_DOWNLOAD ON
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<SOURCE_DIR>
)

ExternalProject_Get_Property(assimp SOURCE_DIR)
set(ASSIMP_INCLUDE_PATH "${SOURCE_DIR}/include")
set(ASSIMP_LIBRARIES "${SOURCE_DIR}/lib/libassimp.so.4")

# glm
ExternalProject_Add(glm
  GIT_REPOSITORY "https://github.com/g-truc/glm.git"
  GIT_TAG "0.9.8"
  LOG_DOWNLOAD ON
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<SOURCE_DIR>
)

ExternalProject_Get_Property(glm SOURCE_DIR)
set(GLM_INCLUDE_PATH "${SOURCE_DIR}/include")

## bullet physics
ExternalProject_Add(bullet
  GIT_REPOSITORY "https://github.com/skylian/bullet3"
  GIT_TAG "master"
  INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/bullet"
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

set(ROBOSCHOOL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/roboschool)

## roboschool
ExternalProject_Add(roboschool
  GIT_REPOSITORY "https://github.com/skylian/roboschool"
  GIT_TAG "remove_qt"
  SOURCE_DIR ${ROBOSCHOOL_DIR}
  DEPENDS assimp bullet
  LOG_DOWNLOAD ON
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  CONFIGURE_COMMAND ""
)

## make roboschool
set(ROBOSCHOOL_INCLUDE_DEPS ${ASSIMP_INCLUDE_PATH} ${GLM_INCLUDE_PATH} ${BULLET_INCLUDE_PATHS})
set(ROBOSCHOOL_LIB_DEPS ${ASSIMP_LIBRARIES} ${BULLET_LIBRARIES})

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
