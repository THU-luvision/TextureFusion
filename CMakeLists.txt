cmake_minimum_required(VERSION 3.10)
project(FlashFusion)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set(GCFUSION_SHADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Shaders" CACHE PATH "Where the shaders live")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)
set(OpenGL_GL_PREFERENCE LEGACY)

find_package(PkgConfig)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT)
find_package(OpenGL)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(Radical)

file(GLOB helloworld_SRC
"*.cpp"
"*.h"
"GCSLAM/*.cpp"
"GCSLAM/*.h"
"GCSLAM/MILD/*.cpp"
"GCSLAM/MILD/*.h"
"GCSLAM/ORBSLAM/*.cpp"
"GCSLAM/ORBSLAM/*.h"
"GCFusion/*.cpp"
"GCFusion/*.h"
"Structure/*.cpp"
"Structure/*.h"
"Shaders/*.cpp"
"Shaders/*.h"
)

add_subdirectory(Tools)
list(APPEND SOURCE_FILES ${helloworld_SRC})
list(APPEND SOURCE_FILES ${WRAPPER_SRC_FILES})

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${TBB_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${SUITESPARSE_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${SOPHUS_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR} )
include_directories(${RADICAL_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "-O3 -w -Wall -msse -msse2 -msse3  -msse4.1 -msse4.2 -mavx2 -DSHADER_DIR=${GCFUSION_SHADER_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -ftree-vectorize -lboost_system -lpthread -lboost_thread -lboost_filesystem")

link_directories(${PROJECT_SOURCE_DIR}/lib)
add_subdirectory(3rd_party)
#add_subdirectory(open_chisel)
include_directories(${WRAPPER_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/3rd_party)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} open_chisel
					  mapmap
                      ${OpenCV_LIBS} 
                      ${Pangolin_LIBRARIES}
		      		  ${OPENGL_LIBRARIES} 
		      		  ${GLUT_LIBRARY}
		      		  ${pthread} 
		      		  ${Boost_LIBRARIES}
		      		  ${TBB_LIBRARIES}
					  ${RADICAL_LIBRARIES}
					  ${WRAPPER_LIBRARY})
#install default configuration file
install(FILES "settings.yaml" DESTINATION ${PROJECT_SOURCE_DIR}/build)
