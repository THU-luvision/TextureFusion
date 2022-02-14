###############################################################################

#add a hint so that it can find it without the pkg-config
find_path(LIBFREENECT2_INCLUDE_DIR libfreenect2.hpp
    PATHS /usr/include/  /usr/local/include
    PATH_SUFFIXES libfreenect2)
#add a hint so that it can find it without the pkg-config
find_library(LIBFREENECT2_LIBRARY
    NAMES libfreenect2.so
    PATHS /usr/lib /usr/local/lib )

  set(LIBFREENECT2_INCLUDE_DIRS ${LIBFREENECT2_INCLUDE_DIR})
  set(LIBFREENECT2_LIBRARIES ${LIBFREENECT2_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LibFreenect2 DEFAULT_MSG
  LIBFREENECT2_LIBRARY LIBFREENECT2_INCLUDE_DIR)

mark_as_advanced(LIBFREENECT2_LIBRARY LIBFREENECT2_INCLUDE_DIR)

