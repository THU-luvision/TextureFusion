file(GLOB_RECURSE CHISEL_srcs *.cpp)
file(GLOB_RECURSE CHISEL_hdrs *.h *.hpp)

add_library(open_chisel SHARED ${CHISEL_srcs} ${CHISEL_hdrs})
target_include_directories(open_chisel PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})