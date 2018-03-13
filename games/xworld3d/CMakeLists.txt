include(roboschool)

add_library(xworld3d
    OBJECT
    xworld3d.cpp
    x3item.cpp
    xworld3d_simulator.cpp
    xworld3d_flags.cpp
)

add_dependencies(xworld3d roboschool)

target_include_directories(xworld3d
    PUBLIC
    ${ROBOSCHOOL_INCLUDE_PATH}
)

include(xworld3d_models)
