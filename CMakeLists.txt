cmake_minimum_required (VERSION 2.6)
project (cs680-labyrinth C CXX)

ADD_DEFINITIONS("-std=c++0x")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/deps)

set(BOX2D_FOUND true)
set(BOX2D_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/Box2D)
ADD_SUBDIRECTORY(lib/Box2D)
include_directories(${BOX2D_INCLUDE_DIR})

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

SET(CMAKE_CXX_FLAGS "")
SET(CMAKE_USE_RELATIVE_PATHS ON)

set(CORELIBS ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${GLEW_LIBRARY})
set(CORELIBS ${CORELIBS} Box2D)

set (LABYRINTH_FILES ${LABYRINTH_FILES} main.cpp)

add_executable(labyrinth ${LABYRINTH_FILES})
target_link_libraries(labyrinth ${CORELIBS})
