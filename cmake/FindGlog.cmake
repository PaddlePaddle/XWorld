# - Try to find Glog
#
# The following variables are optionally searched for defaults
#  GLOG_ROOT_DIR:            Base directory where all GLOG components are found
#
# The following are set after configuration is done:
#  GLOG_FOUND
#  GLOG_INCLUDE_DIRS
#  GLOG_LIBRARIES

include(FindPackageHandleStandardArgs)

set(GLOG_ROOT_DIR "" CACHE PATH "Folder contains Google glog")

if (NOT GLOG_ROOT_DIR)
  find_path(GLOG_INCLUDE_DIR glog/logging.h)
  find_library(GLOG_LIBRARY glog
    PATH_SUFFIXES lib lib64)
else()
  find_path(GLOG_TMP_INCLUDE_DIR glog/logging.h
    PATHS ${GLOG_ROOT_DIR}/include
    NO_DEFAULT_PATH)
  set(GLOG_INCLUDE_DIR ${GLOG_TMP_INCLUDE_DIR})
  find_library(GLOG_TMP_LIBRARY glog
    PATHS ${GLOG_ROOT_DIR}
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib lib64)
  set(GLOG_LIBRARY ${GLOG_TMP_LIBRARY})
endif()

## set GLOG_FOUND
find_package_handle_standard_args(Glog DEFAULT_MSG GLOG_INCLUDE_DIR GLOG_LIBRARY)

if(GLOG_FOUND)
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
  message(STATUS "Found glog (include: ${GLOG_INCLUDE_DIR}, library: ${GLOG_LIBRARY})")
  mark_as_advanced(GLOG_ROOT_DIR GLOG_LIBRARY GLOG_INCLUDE_DIR)
endif()
