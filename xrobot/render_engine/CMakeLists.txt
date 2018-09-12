find_package(OpenGL REQUIRED)

include(assimp)
include(glm)
include(glfw)

add_library(render_engine OBJECT 
    render_utils.cpp
    csm.cpp
    fbo.cpp
    shader.cpp
    camera.cpp
    mesh.cpp
    model.cpp
    gl_context.cpp
    render.cpp
    render_world.cpp
    render2d.cpp
    texture3d.cpp)

add_dependencies(render_engine assimp glm glfw) 

set(EGL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
# set(EGL_LIBRARIES "/usr/lib/x86_64-linux-gnu/libEGL.so")
set(EGL_LIBRARIES "/usr/lib/nvidia-396/libEGL.so")


target_include_directories(render_engine PUBLIC
    ${EGL_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${EGL_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS})

set(DEP_LIBS
    ${DEP_LIBS}
    ${EGL_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${OPENGL_gl_LIBRARY}
    ${GLFW_LIBRARIES}
    ${BULLET_LIBRARIES}
    PARENT_SCOPE)

set(DEP_INCLUDES
    ${DEP_INCLUDES}
    ${EGL_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    PARENT_SCOPE)
