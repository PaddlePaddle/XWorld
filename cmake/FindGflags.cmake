# - Try to find Gflags
#
# The following variables are optionally searched for defaults
#  GFLAGS_ROOT_DIR:            Base directory where all GFLAGS components are found
#
# The following are set after configuration is done:
#  GFLAGS_FOUND
#  GFLAGS_INCLUDE_DIRS
#  GFLAGS_LIBRARIES

include(FindPackageHandleStandardArgs)

set(GFLAGS_ROOT_DIR "" CACHE PATH "Folder contains Google gflags")

if (NOT GFLAGS_ROOT_DIR)
  find_path(GFLAGS_INCLUDE_DIR gflags/gflags.h)
  find_library(GFLAGS_LIBRARY gflags
    PATH_SUFFIXES lib lib64)
else()
  find_path(GFLAGS_TMP_INCLUDE_DIR gflags/gflags.h
    PATHS ${GFLAGS_ROOT_DIR}/include
    NO_DEFAULT_PATH)
  set(GFLAGS_INCLUDE_DIR ${GFLAGS_TMP_INCLUDE_DIR})
  find_library(GFLAGS_TMP_LIBRARY gflags
    PATHS ${GFLAGS_ROOT_DIR}
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib lib64)
  set(GFLAGS_LIBRARY ${GFLAGS_TMP_LIBRARY})
endif()

## set GFLAGS_FOUND
find_package_handle_standard_args(Gflags DEFAULT_MSG GFLAGS_INCLUDE_DIR GFLAGS_LIBRARY)

if(GFLAGS_FOUND)
  set(GFLAGS_INCLUDE_DIRS ${GFLAGS_INCLUDE_DIR})
  set(GFLAGS_LIBRARIES ${GFLAGS_LIBRARY})
  message(STATUS "Found gflags (include: ${GFLAGS_INCLUDE_DIR}, library: ${GFLAGS_LIBRARY})")
  mark_as_advanced(GFLAGS_ROOT_DIR GFLAGS_LIBRARY GFLAGS_INCLUDE_DIR)
endif()
