find_path(MAPMAP_INCLUDE_DIRS 
		  NAMES full.h
          PATHS
            "${PROJECT_SOURCE_DIR}/libs/mapmap"
            /usr/include
	    /usr/local/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MapMap DEFAULT_MSG
 MAPMAP_INCLUDE_DIRS)

mark_as_advanced(MAPMAP_INCLUDE_DIRS)
