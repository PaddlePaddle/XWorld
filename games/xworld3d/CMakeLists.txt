function(append_libraries lib_list library_dir libraries)
    set(tmp_list "")
    foreach(lib ${libraries})
        list(APPEND tmp_list "${library_dir}/lib${lib}.so")
    endforeach(lib)
    set(${lib_list} ${tmp_list} PARENT_SCOPE)
endfunction(append_libraries)

set(ROBOSCHOOL_INCLUDE_PATH "${ROBOSCHOOL_ROOT}/cpp-household")
set(ROBOSCHOOL_LIB "${ROBOSCHOOL_ROOT}/libroboschool.so")
set(BULLET_ROOT_DIR "${ROBOSCHOOL_ROOT}/cpp-household/bullet_local_install")

add_library(xworld3d
    OBJECT
    x3scene.cpp
    x3item.cpp
    xworld3d_simulator.cpp
    xworld3d_flags.cpp
    xworld3d_parser.cpp
)

target_include_directories(xworld3d
    PUBLIC
    ${ROBOSCHOOL_INCLUDE_PATH}
)

set(DEP_LIBS ${ROBOSCHOOL_LIB})

find_package(PkgConfig)

## Bullet Physics
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH} "${BULLET_ROOT_DIR}/lib/cmake/bullet"
)

include(BulletConfig)

target_include_directories(xworld3d
    PUBLIC
    "${BULLET_ROOT_DIR}/include"
    "${BULLET_ROOT_DIR}/include/bullet"
)

set(bullet_libs "")
append_libraries(bullet_libs "${BULLET_ROOT_DIR}/lib" "${BULLET_LIBRARIES}")

set(DEP_LIBS
    ${DEP_LIBS}
    ${bullet_libs}
)

## Assimp 
pkg_check_modules(Assimp REQUIRED assimp>=4.0)

target_include_directories(xworld3d
    PUBLIC
    "${Assimp_INCLUDE_DIRS}"
)

set(assimp_libs "")
append_libraries(assimp_libs "${Assimp_LIBRARY_DIRS}" "${Assimp_LIBRARIES}")

set(DEP_LIBS
    ${DEP_LIBS}
    ${assimp_libs}
)

## Qt5
pkg_check_modules(Qt5 REQUIRED Qt5Widgets>=5.5 Qt5OpenGL>=5.5)

target_include_directories(xworld3d
    PUBLIC
    ${Qt5_INCLUDE_DIRS}
)

set(qt5_libs "")
append_libraries(qt5_libs "${Qt5_LIBRARY_DIRS}" "${Qt5_LIBRARIES}")

set(DEP_LIBS
    ${DEP_LIBS}
    ${qt5_libs}
)
