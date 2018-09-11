message("External project - Assimp")

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
set(ASSIMP_INCLUDE_DIRS "${INSTALL_DIR}/include")
set(ASSIMP_LIBRARIES "${INSTALL_DIR}/lib/libassimp.so.4")
