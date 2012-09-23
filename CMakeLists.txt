cmake_minimum_required (VERSION 2.6)
project (cs680-labyrinth C CXX)

ADD_DEFINITIONS("-std=c++0x")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/deps)

find_package(GLUT)
find_package(OpenGL)
find_package(GLEW)


SET(CMAKE_CXX_FLAGS "")
SET(CMAKE_USE_RELATIVE_PATHS ON)

set(CORELIBS ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${GLEW_LIBRARY})

set (LABYRINTH_FILES ${LABYRINTH_FILES} src/main.cpp)

add_executable(../bin/labyrinth ${LABYRINTH_FILES})
target_link_libraries(../bin/labyrinth ${CORELIBS})
