cmake_minimum_required (VERSION 2.6)
ADD_DEFINITIONS("-std=c++0x")
SET(CMAKE_CXX_FLAGS "")
SET(CMAKE_USE_RELATIVE_PATHS ON)

set(CORELIBS ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${GLEW_LIBRARY})
set(CORELIBS ${CORELIBS} Box2D)

set (LABYRINTH_FILES ${LABYRINTH_FILES} main.cpp)

file(MAKE_DIRECTORY ../bin/)
add_executable(../../bin/labyrinth ${LABYRINTH_FILES})
target_link_libraries(../../bin/labyrinth ${CORELIBS})
