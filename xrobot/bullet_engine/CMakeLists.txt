add_library(bullet_engine OBJECT 
    common.h
    common.cpp
    bullet_joint.cpp
    bullet_object.cpp
    bullet_body.cpp
    bullet_world.cpp
)

add_dependencies(bullet_engine assimp bullet glfw glm utils)

target_include_directories(bullet_engine PUBLIC
    ${ASSIMP_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS})

set(DEP_INCLUDES
    ${DEP_INCLUDES}
    PARENT_SCOPE)
