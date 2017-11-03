include(roboschool)

add_library(xworld3d
    OBJECT
    x3scene.cpp
    x3item.cpp
    xworld3d_simulator.cpp
    xworld3d_flags.cpp
    xworld3d_parser.cpp
)

add_dependencies(xworld3d roboschool)

target_include_directories(xworld3d
    PUBLIC
    ${ROBOSCHOOL_INCLUDE_PATH}
)
