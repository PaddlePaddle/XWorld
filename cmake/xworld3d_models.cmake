message("External project - xworld3d model zoo")

ExternalProject_Add(xworld3d_model_zoo
  GIT_REPOSITORY "https://github.com/yu239/XWorld3D-model-zoo"
  GIT_TAG "master"
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/xworld3d_model_zoo
  LOG_DOWNLOAD ON
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  CONFIGURE_COMMAND ""
)

ExternalProject_Get_Property(xworld3d_model_zoo SOURCE_DIR)

add_custom_command(TARGET xworld3d_model_zoo POST_BUILD
  COMMAND unzip -o -q ${SOURCE_DIR}/models_3d.zip
  COMMAND unzip -o -q ${SOURCE_DIR}/glsl.zip
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Download xworld3d models"
)
