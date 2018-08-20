message("External project - bullet")

## bullet physics
ExternalProject_Add(bullet
  PREFIX ${EXTERNAL_PROJECT_PREFIX}
  GIT_REPOSITORY "http://github.com/ziyuli/bullet3.git"
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
set(BULLET_INCLUDE_DIRS "${INSTALL_DIR}/include" "${INSTALL_DIR}/include/bullet")
set(BULLET_LIBRARIES
  "${INSTALL_DIR}/lib/libLinearMath.so"
  "${INSTALL_DIR}/lib/libBullet3Common.so"
  "${INSTALL_DIR}/lib/libBulletCollision.so"
  "${INSTALL_DIR}/lib/libBulletDynamics.so"
  "${INSTALL_DIR}/lib/libBulletInverseDynamics.so"
  "${INSTALL_DIR}/lib/libPhysicsClientC_API.so")
