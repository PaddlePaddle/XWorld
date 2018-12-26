find_package(OpenGL REQUIRED)

add_library(render_engine OBJECT 
    render_utils.cpp
    shader.cpp
    frustum.cpp
    camera.cpp
    mesh.cpp
    gl_context.cpp
    visualization.cpp
    csm.cpp
    postprocess.cpp
    composite.cpp
    ssao.cpp
    ssr.cpp
    fxaa.cpp
    vct.cpp
    render.cpp
    render_target.cpp
    render_world.cpp
    texture3d.cpp
    texture2d.cpp
    capture.cpp
    model.cpp)

add_dependencies(render_engine assimp glm glfw utils) 

find_library(EGL_LIBRARY EGL PATHS
  /usr/local/lib
  /usr/lib)

if(NOT EGL_LIBRARY) 
  set(EGL_LIBRARY "")
  set(EGL_INCLUDE_DIRS "")
else()
  add_definitions(-DUSE_EGL)
  set(EGL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

target_include_directories(render_engine PUBLIC
  ${ASSIMP_INCLUDE_DIRS}
  ${EGL_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS})

set(DEP_LIBS
  ${DEP_LIBS}
  ${EGL_LIBRARY}
  ${ASSIMP_LIBRARIES}
  ${OPENGL_gl_LIBRARY}
  ${GLFW_LIBRARIES}
  PARENT_SCOPE)

set(DEP_INCLUDES
  ${DEP_INCLUDES}
  ${EGL_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIRS}
  PARENT_SCOPE)
