cmake_minimum_required(VERSION 3.14)
#set(CMAKE_VERBOSE_MAKEFILE ON)

set(PROJECT_NAME "autocraft")
project(${PROJECT_NAME})

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

#################################################
#              Dependencies                     #
#################################################
#pybind11
if (PYBIND11_DIR)
  set(PYBIND11_ROOT_DIR ${PYBIND11_DIR})
else()
  set(PYBIND11_ROOT_DIR $ENV{PYBIND11_DIR})
endif()
message(STATUS "pybind11: ${PYBIND11_ROOT_DIR}")
unset(PYBIND11_DIR CACHE)

add_subdirectory(${PYBIND11_ROOT_DIR} "./pybind11")

# boost
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.6 REQUIRED COMPONENTS system graph)

# Python
find_package(PythonLibs REQUIRED)


# yaml-cpp
if (YAML_CPP_DIR)
  set(YAML_CPP_ROOT_DIR ${YAML_CPP_DIR})
else()
  set(YAML_CPP_ROOT_DIR $ENV{YAML_CPP_DIR})
endif()
message(STATUS "yaml-cpp: ${YAML_CPP_ROOT_DIR}")
unset(YAML_CPP_DIR CACHE)

# z3
#find_package(Z3 CONFIG REQUIRED)
if (Z3_DIR)
  set(Z3_ROOT_DIR ${Z3_DIR})
else()
	set(Z3_ROOT_DIR $ENV{Z3_DIR})
endif()
message(STATUS "z3: ${Z3_ROOT_DIR}")
unset(Z3_DIR CACHE)

# spdlog
if (SPDLOG_DIR)
	set(SPDLOG_ROOT_DIR ${SPDLOG_DIR})
else()
	set(SPDLOG_ROOT_DIR $ENV{SPDLOG_DIR})
endif()
message(STATUS "spdlog: ${SPDLOG_ROOT_DIR}")
unset(SPDLOG_DIR CACHE)

# phmap
if (PHMAP_DIR)
  set(PHMAP_ROOT_DIR ${PHMAP_DIR})
else()
  set(PHMAP_ROOT_DIR $ENV{PHMAP_DIR})
endif()
message(STATUS "phmap: ${PHMAP_ROOT_DIR}")
unset(PHMAP_DIR CACHE)

# nanoflann
if (NANOFLANN_DIR)
  set(NANOFLANN_ROOT_DIR ${NANOFLANN_DIR})
else()
  set(NANOFLANN_ROOT_DIR $ENV{NANOFLANN_DIR})
endif()
message(STATUS "nanoflann: ${NANOFLANN_ROOT_DIR}")
unset(NANOFLANN_DIR CACHE)

# limbo
if (LIMBO_DIR)
  set(LIMBO_ROOT_DIR ${LIMBO_DIR})
else()
  set(LIMBO_ROOT_DIR $ENV{LIMBO_DIR})
endif()
message(STATUS "limbo: ${LIMBO_ROOT_DIR}")
unset(LIMBO_DIR CACHE)

# lefdef
if (LEF_DIR)
  set(LEF_ROOT_DIR ${LEF_DIR})
else()
  set(LEF_ROOT_DIR $ENV{LEF_DIR})
endif()
message(STATUS "lef: ${LEF_ROOT_DIR}")
unset(LEF_DIR CACHE)
if (DEF_DIR)
  set(DEF_ROOT_DIR ${DEF_DIR})
else()
  set(DEF_ROOT_DIR $ENV{DEF_DIR})
endif()
message(STATUS "def: ${DEF_ROOT_DIR}")
unset(DEF_DIR CACHE)

# lemon
if (LEMON_DIR)
  set(LEMON_ROOT_DIR ${LEMON_DIR})
else()
  set(LEMON_ROOT_DIR $ENV{LEMON_DIR})
endif()
message(STATUS "lemon: ${LEMON_ROOT_DIR}")
unset(LEMON_DIR CACHE)

# cxxopts
if (CXXOPTS_DIR)
  set(CXXOPTS_ROOT_DIR ${CXXOPTS_DIR})
else()
  set(CXXOPTS_ROOT_DIR $ENV{CXXOPTS_DIR})
endif()
message(STATUS "cxxopts: ${CXXOPTS_ROOT_DIR}")
unset(CXXOPTS_DIR CACHE)

# zlib
if (ZLIB_DIR)
  set(ZLIB_ROOT_DIR ${ZLIB_DIR})
else()
  set(ZLIB_ROOT_DIR $ENV{ZLIB_DIR})
endif()
message(STATUS "zlib: ${ZLIB_ROOT_DIR}")
unset(ZLIB_DIR CACHE)

#################################################
  

# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "Using build type ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "-std=c++2a -Wall -fopenmp -fPIC -fconcepts -lz")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g -fno-inline")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_EXE_LINKER_FLAGS "-pthread")

add_definitions(-DSPDLOG_COMPILED_LIB)

#################################################
#           Include library path
#################################################
include_directories (
  ${CMAKE_CURRENT_SOURCE_DIR}/src
	${SPDLOG_ROOT_DIR}/include
  ${YAML_CPP_ROOT_DIR}/include
  ${SPDLOG_ROOT_DIR}/include
  ${PHMAP_ROOT_DIR}/include
  ${NANOFLANN_ROOT_DIR}/include
  ${LIMBO_ROOT_DIR}/include
  ${LEF_ROOT_DIR}/include
  ${DEF_ROOT_DIR}/include
  ${Z3_ROOT_DIR}/include
  ${LEMON_ROOT_DIR}/include
  ${CXXOPTS_ROOT_DIR}/include
)

file(GLOB LIB
	${SPDLOG_ROOT_DIR}/lib/libspdlog.a
  ${YAML_CPP_ROOT_DIR}/lib/libyaml-cpp.a
  ${LIMBO_ROOT_DIR}/lib/libgdsdb.a
  ${LIMBO_ROOT_DIR}/lib/libgdsparser.a
  ${LIMBO_ROOT_DIR}/lib/libgzstream.a
  ${LEF_ROOT_DIR}/lib/liblef.a
  ${DEF_ROOT_DIR}/lib/libdef.a
  ${Z3_ROOT_DIR}/lib/libz3.a
  ${LEMON_ROOT_DIR}/lib/libemon.a
  #${ZLIB_DIR}/lib/libz.a
)

link_libraries (
  ${PYTHON_LIBRARIES}
)


###############################################
#                 SRC
###############################################
set(SRC
	src/thirdparty/spdlog.cpp
  src/util/util.cpp
  src/io/parser.cpp
  src/io/parHspice.cpp
  src/io/parLef.cpp
  src/io/parGds.cpp
  src/io/writer.cpp
  src/db/dbBasic.cpp
  src/db/dbCell.cpp
  src/db/dbCir.cpp
  src/db/dbNet.cpp
  src/db/dbLayer.cpp
  src/place/placeMgr.cpp
  src/place/placeSMT.cpp
  src/route/routeMgr.cpp
  # src/route/routeGrMgr.cpp
  src/route/routeDrMgr.cpp
  src/route/routeDrPS.cpp
  src/route/routeDrAstar.cpp
  src/route/routeDrPost.cpp
  src/drc/drcMgr.cpp
  src/rl/rlUtil.cpp
)

# executable
add_executable(${PROJECT_NAME}E ${SRC} src/main/main.cpp)
#target_link_libraries(${PROJECT_NAME}E PUBLIC Boost::system Boost::graph ${LIB} ortools::ortools z3::libz3)
target_link_libraries(${PROJECT_NAME}E PUBLIC Boost::system Boost::graph ${LIB})

# Add modules to pybind
pybind11_add_module(${PROJECT_NAME} ${SRC} src/api/api.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES} ${LIB})

