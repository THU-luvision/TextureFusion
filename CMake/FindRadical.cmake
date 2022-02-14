#add a hint so that it can find it without the pkg-config
find_path(RADICAL_INCLUDE_DIR radical
    HINTS /usr/include/  /usr/local/include)
#add a hint so that it can find it without the pkg-config
find_library(RADICAL_LIBRARY
    NAMES libradical.so
    HINTS /usr/lib /usr/local/lib )

  set(RADICAL_INCLUDE_DIRS ${RADICAL_INCLUDE_DIR})
  set(RADICAL_LIBRARIES ${RADICAL_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Radical DEFAULT_MSG
  RADICAL_LIBRARY RADICAL_INCLUDE_DIR)

mark_as_advanced(RADICAL_LIBRARY RADICAL_INCLUDE_DIR)

