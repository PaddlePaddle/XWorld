find_package(OpenGL REQUIRED)

include(assimp)
include(glm)
include(glfw)

add_library(render_engine OBJECT 
    render_utils.cpp
    fbo.cpp
    shader.cpp
    camera.cpp
    mesh.cpp
    model.cpp
    gl_context.cpp
    render.cpp
    render2d.cpp)

add_dependencies(render_engine assimp glm glfw) 

target_include_directories(render_engine PUBLIC
    ${ASSIMP_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS})

set(DEP_LIBS
    ${DEP_LIBS}
    "/usr/lib/x86_64-linux-gnu/libEGL.so"
    ${ASSIMP_LIBRARIES}
    ${OPENGL_gl_LIBRARY}
    ${GLFW_LIBRARIES}
    ${BULLET_LIBRARIES}
    PARENT_SCOPE)

set(DEP_INCLUDES
    ${DEP_INCLUDES}
    ${ASSIMP_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    PARENT_SCOPE)
