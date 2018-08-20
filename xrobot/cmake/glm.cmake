message("External project - glm")
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
set(GLM_INCLUDE_DIRS "${INSTALL_DIR}/include")
