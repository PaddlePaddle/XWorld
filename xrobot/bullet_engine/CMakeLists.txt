add_library(bullet_engine OBJECT 
  common.cpp
  bullet_joint.cpp
  bullet_object.cpp
  bullet_body.cpp
  bullet_world.cpp)

add_dependencies(bullet_engine bullet glm utils)

target_include_directories(bullet_engine PUBLIC
  ${BULLET_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIRS})

set(DEP_LIBS
  ${DEP_LIBS}
  ${BULLET_LIBRARIES}
  PARENT_SCOPE)

set(DEP_INCLUDES
  ${DEP_INCLUDES}
  ${BULLET_INCLUDE_DIRS}
  PARENT_SCOPE)
