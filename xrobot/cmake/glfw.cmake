message("External project - glfw")

## bullet physics
ExternalProject_Add(glfw
  PREFIX ${EXTERNAL_PROJECT_PREFIX}
  GIT_REPOSITORY "https://github.com/glfw/glfw.git"
  GIT_TAG "master"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/external/glfw"
  INSTALL_DIR "${EXTERNAL_PROJECT_PREFIX}/glfw"
  LOG_DOWNLOAD ON
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
)

ExternalProject_Get_Property(glfw INSTALL_DIR)
set(GLFW_INCLUDE_DIRS "${INSTALL_DIR}/include")
set(GLFW_LIBRARIES "${INSTALL_DIR}/lib/libglfw3.a")
