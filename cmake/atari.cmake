message("External project - atari")

ExternalProject_Add(atari
  GIT_REPOSITORY "https://github.com/skylian/Arcade-Learning-Environment.git"
  GIT_TAG "master"
  INSTALL_COMMAND ""
  CMAKE_ARGS
    -DUSE_SDL=OFF
    -DBUILD_CLI=OFF
    -DBUILD_EXAMPLES=OFF
    -DBUILD_C_LIB=OFF
)

ExternalProject_Get_Property(atari SOURCE_DIR)
set(ATARI_INCLUDE_PATH "${SOURCE_DIR}/src")
set(DEP_LIBS ${DEP_LIBS} "${SOURCE_DIR}/libale.so")
