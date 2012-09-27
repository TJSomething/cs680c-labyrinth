cmake_minimum_required (VERSION 2.6)
project (cs680-labyrinth C CXX)

ADD_DEFINITIONS("-std=c++0x")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/deps)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Box2D REQUIRED)

SET(CMAKE_CXX_FLAGS "")
SET(CMAKE_USE_RELATIVE_PATHS ON)

set(CORELIBS ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${GLEW_LIBRARY})
set(CORELIBS ${CORELIBS} ${BOX2D_LIBRARY})

set (LABYRINTH_FILES ${LABYRINTH_FILES} src/main.cpp)

add_executable(../bin/labyrinth ${LABYRINTH_FILES})
target_link_libraries(../bin/labyrinth ${CORELIBS})
