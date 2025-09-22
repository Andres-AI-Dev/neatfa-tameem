cmake_minimum_required(VERSION 3.5)
project(iAnt-ARGoS)

# Deactivate RPATH for MacOSX.
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package.
set(ARGOS_CMAKE_DIR "/opt/homebrew/share/argos3/cmake")
find_package(ARGoS REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check for required libraries to compile code depending on Qt and OpenGL.
include(FindARGoSQTOpenGL)

# Find the GSL package.
find_package(GSL REQUIRED)

# Find the LUA package.
find_package(Lua REQUIRED)

# Find jsoncpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# Set ARGoS include directory.
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${LUA_INCLUDE_DIR} ${JSONCPP_INCLUDE_DIRS})

# Set ARGoS link directory.
link_directories(/opt/homebrew/lib/argos3 ${JSONCPP_LIBRARY_DIRS})
link_libraries(${ARGOS_LDFLAGS})

# Descend into the source code directory.
add_subdirectory(source)

# pybind11 module build
find_package(pybind11 QUIET)
if(pybind11_FOUND)
  message(STATUS "pybind11 found, building Python module iant_rl")
  pybind11_add_module(iant_rl source/py_iant_rl.cpp)
  # Disable Qt automoc for this pure-Python module to avoid moc interfering
  set_target_properties(iant_rl PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
  target_include_directories(iant_rl PRIVATE ${ARGOS_INCLUDE_DIRS})
  target_link_libraries(iant_rl PRIVATE
    iAnt_loop_functions
    iAnt_controller
    argos3core_simulator
    argos3plugin_simulator_entities
    argos3plugin_simulator_footbot
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_qtopengl
  )
else()
  message(WARNING "pybind11 not found, skipping Python module iant_rl")
endif()

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
