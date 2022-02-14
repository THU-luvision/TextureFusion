set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)

option(WITH_OPENNI2 "Build with Openni2 support" ON)
option(WITH_REALSENSE2 "Build with Realsense2 support" ON)
option(WITH_FREENECT2 "Build with Freenect2 support" OFF)

file(GLOB WRAPPER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
								  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
								  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

if(WITH_OPENNI2)
	find_package(OpenNI2)
	list(APPEND WRAPPER_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})
	list(APPEND WRAPPER_LIBRARIES ${OPENNI2_LIBRARY})
	file(GLOB_RECURSE OPENNI2_SRC "${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/*.h"
								  "${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/*.cpp"
								  "${CMAKE_CURRENT_SOURCE_DIR}/OpenNI2/*.hpp")
	list(APPEND WRAPPER_SRC ${OPENNI2_SRC})
else()
	message(STATUS "OpenNI2 support: disabled")
endif()

if(WITH_REALSENSE2)
	find_package(RealSense2)
	list(APPEND WRAPPER_INCLUDE_DIRS ${LIBREALSENSE_INCLUDE_DIR})
	list(APPEND WRAPPER_LIBRARIES ${LIBREALSENSE_LIBRARY})
	file(GLOB_RECURSE REALSENSE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/RealSense/*.h"
									"${CMAKE_CURRENT_SOURCE_DIR}/RealSense/*.cpp"
									"${CMAKE_CURRENT_SOURCE_DIR}/RealSense/*.hpp")
	list(APPEND WRAPPER_SRC ${REALSENSE_SRC})
else()
	message(STATUS "Realsense support: disabled")
endif()

if(WITH_FREENECT2)
	find_package(FreeNect2)
	list(APPEND WRAPPER_INCLUDE_DIRS ${LIBFREENECT2_INCLUDE_DIR})
	list(APPEND WRAPPER_LIBRARIES ${LIBFREENECT2_LIBRARY})
	file(GLOB_RECURSE KINECT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/Kinect/*.h"
								 "${CMAKE_CURRENT_SOURCE_DIR}/Kinect/*.cpp"
								 "${CMAKE_CURRENT_SOURCE_DIR}/Kinect/*.hpp")
	list(APPEND WRAPPER_SRC ${KINECT_SRC})
else()
	message(STATUS "Freenect2 support: disabled")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/config.h)

set(WRAPPER_SRC_FILES ${WRAPPER_SRC} CACHE INTERNAL "Source code of Wrappers")
set(WRAPPER_INCLUDE_DIR ${WRAPPER_INCLUDE_DIRS} CACHE INTERNAL "Include directories of Wrappers")
set(WRAPPER_LIBRARY ${WRAPPER_LIBRARIES} CACHE INTERNAL "Libraries of Wrappers")
