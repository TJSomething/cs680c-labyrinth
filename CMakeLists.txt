cmake_minimum_required (VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/deps)

set(LIB_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(BOX2D_BUILD_EXAMPLES OFF)

set(BOX2D_FOUND true)

set(BOX2D_INCLUDE_DIR ${LIB_ROOT}/Box2D)
ADD_SUBDIRECTORY(${LIB_ROOT}/Box2D)
include_directories(${BOX2D_INCLUDE_DIR})

set(ASSIMP_INCLUDE_DIR ${LIB_ROOT}/assimp/include)
ADD_SUBDIRECTORY(${LIB_ROOT}/assimp)
include_directories(${ASSIMP_INCLUDE_DIR})

set(STB_IMAGE_INCLUDE_DIR ${LIB_ROOT}/stb_image)
ADD_SUBDIRECTORY(${LIB_ROOT}/stb_image)
include_directories(${STB_IMAGE_INCLUDE_DIR})

find_package(PNG REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
FIND_PACKAGE (Boost COMPONENTS system filesystem REQUIRED)

add_subdirectory(src)