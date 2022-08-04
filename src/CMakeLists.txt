

FILE(GLOB_RECURSE PROG_SRC  *.cpp)
FILE(GLOB_RECURSE PROG_SRC2  *.cc)
FILE(GLOB_RECURSE PROG_SRC_C  *.c)
FILE(GLOB_RECURSE PROG_HEADER  *.h)
FILE(GLOB_RECURSE CUDA_SRC  *.cu)


SET(PROG_SRC ${PROG_SRC} ${PROG_SRC2} ${PROG_SRC_C} ${CUDA_SRC})

#creates 'folders' in the solution explorer of visual studio
GroupSources2(${PROJECT_SOURCE_DIR}/src)


include_directories(.)
include_directories(cuda)
include_directories(${PACKAGE_INCLUDES})

add_executable(${PROG_NAME} ${PROG_SRC} ${PROG_HEADER})

set(LIBS ${LIBS} ${LIB_TARGETS})
message(STATUS "Libs: ${LIBS}")
target_link_libraries(${PROG_NAME} ${LIBS} )




