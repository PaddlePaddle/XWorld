message("External project - OpenCV 3.2.0")

ExternalProject_Add(opencv
  GIT_REPOSITORY "https://github.com/opencv/opencv.git"
  GIT_TAG 3.2.0
  INSTALL_DIR "${CMAKE_SOURCE_DIR}/external/opencv"
  LOG_DOWNLOAD ON
  UPDATE_COMMAND ""
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_DOCS=OFF
    -DBUILD_PACKAGE=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_FAT_JAVA_LIB=OFF
    -DBUILD_PERF_TESTS=OFF
    -DBUILD_EXAMPLES=OFF
    -DCUDA_GENERATION=Auto
    -DWITH_IPP=OFF
    -DWITH_LAPACK=OFF
    -DWITH_QTKIT=OFF
    -DWITH_OPENGL=OFF
    -DWITH_OPENNI=OFF
    -DWITH_OPENVX=OFF
    -DWITH_INTELPERC=OFF
    -DWITH_GPHOTO2=OFF
    -DWITH_VTK=OFF
    -DWITH_CUDA=OFF
)

ExternalProject_Get_Property(opencv INSTALL_DIR)
set(OPENCV_INCLUDE_PATH "${INSTALL_DIR}/include")
set(OPENCV_LIBS "")
set(LIB_NAMES opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs)
foreach(LIB ${LIB_NAMES})
  set(OPENCV_LIBS ${OPENCV_LIBS} "${INSTALL_DIR}/lib/lib${LIB}.so")
endforeach()

set(OPENCV_INSTALL_DIR ${INSTALL_DIR})
include_directories(${OPENCV_INCLUDE_PATH})
