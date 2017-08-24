message("External project - deepmind_lab")

ExternalProject_Add(deepmind_lab
  GIT_REPOSITORY "https://github.com/yu239/lab.git"
  GIT_TAG "master"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_DIR ""
  INSTALL_COMMAND ""
)

ExternalProject_Get_Property(deepmind_lab SOURCE_DIR)

set(DEEPMIND_LAB_PATH ${SOURCE_DIR})

ExternalProject_Add_Step(deepmind_lab bazel_build
  COMMAND bazel build :deepmind_lab.so --define headless=glx
  WORKING_DIRECTORY ${DEEPMIND_LAB_PATH}
  DEPENDEES download
  DEPENDERS build
  )

## copy run files
set(COMMAND
  "${CMAKE_CURRENT_SOURCE_DIR}/export_vars.sh \
   DEEPMIND_RUNFILES ${SOURCE_DIR}/bazel-bin/deepmind_lab.so.runfiles/org_deepmind_lab")
execute_process(COMMAND bash "-c" ${COMMAND})

## in-source build
set(DEEPMIND_LAB_INCLUDE_PATH "${SOURCE_DIR}")
set(DEP_LIBS ${DEP_LIBS} "${SOURCE_DIR}/bazel-bin/libdmlab.so")
