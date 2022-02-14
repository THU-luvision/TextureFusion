# -*- mode: cmake; -*-
###############################################################################
# Find librealsense  https://github.com/IntelRealSense/librealsense
#
# This sets the following variables:
# LIBREALSENSE_FOUND - True if OPENNI was found.
# LIBREALSENSE_INCLUDE_DIRS - Directories containing the OPENNI include files.
# LIBREALSENSE_LIBRARIES - Libraries needed to use OPENNI.
# LIBREALSENSE_DEFINITIONS - Compiler flags for OPENNI.
#
# File forked from augmented_dev, project of alantrrs
# (https://github.com/alantrrs/augmented_dev).

find_package(PkgConfig)
#add a hint so that it can find it without the pkg-config
find_path(LIBREALSENSE_INCLUDE_DIR rs.h
    HINTS /usr/include/  /usr/local/include
    PATH_SUFFIXES librealsense2)
#add a hint so that it can find it without the pkg-config
find_library(LIBREALSENSE_LIBRARY
    NAMES librealsense2.so
    HINTS /usr/lib /usr/local/lib )

  set(LIBREALSENSE_INCLUDE_DIRS ${LIBREALSENSE_INCLUDE_DIR})
  set(LIBREALSENSE_LIBRARIES ${LIBREALSENSE_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RealSense2 DEFAULT_MSG
  LIBREALSENSE_LIBRARY LIBREALSENSE_INCLUDE_DIR)

mark_as_advanced(LIBREALSENSE_LIBRARY LIBREALSENSE_INCLUDE_DIR)

