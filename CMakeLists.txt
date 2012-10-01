cmake_minimum_required (VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/deps)

set(BOX2D_BUILD_EXAMPLES OFF)

set(BOX2D_FOUND true)
set(BOX2D_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/Box2D)
ADD_SUBDIRECTORY(lib/Box2D)
include_directories(${BOX2D_INCLUDE_DIR})

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

add_subdirectory(src)