project(BAMapping)
cmake_minimum_required(VERSION 2.8)

set (CMAKE_CXX_STANDARD 11)


#find_path(CHOLMOD_INCLUDE_DIR
#  NAMES
#  cholmod.h
#  PATHS
#  $ENV{CHOLMODDIR}
#  ${INCLUDE_INSTALL_DIR}
#  PATH_SUFFIXES
#  suitesparse
#  ufsparse
#)
#include_directories(${CHOLMOD_INCLUDE_DIR})
#find_library(CHOLMOD_LIBRARY
#    NAMES cholmod libcholmod
#    PATHS $ENV{CHOLMODDIR} ${LIB_INSTALL_DIR})

#set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARY})


find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


#include_directories(${GFLAGS_INCLUDE_DIRS})
#add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(CPUTSDF REQUIRED PATHS third_party/cpu_tsdf/build)
include_directories(${CPUTSDF_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED
            src/util/Optimize.cpp
            src/util/Frame.cpp
            src/util/Point.cpp
            src/util/Graph.cpp
            src/util/BundleAdjuster.cpp
            src/util/bal_problem.cpp
            src/Viewer.cpp
            src/Reader.cpp
            src/Writer.cpp)
#add_library(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
                                 ceres
                                 ${CPUTSDF_LIBRARIES}
#                                 ${CHOLMOD_LIBRARIES}
#                                 ${GFLAGS_LIBRARIES}
                                 ${Pangolin_LIBRARIES})

add_executable(System src/System.cpp)
#                        src/util/Optimize.cpp
#                        src/util/Frame.cpp
#                        src/util/Point.cpp
#                        src/util/Graph.cpp
#                        src/util/BundleAdjuster.cpp
#                        src/util/bal_problem.cpp
#                        src/Viewer.cpp
#                        src/Reader.cpp)
target_link_libraries(System ${PROJECT_NAME})

add_executable(test_bundleAdjuster src/tests/test_bundleAdjuster.cpp)
#                                    src/util/Optimize.cpp
#                                    src/util/Frame.cpp
#                                    src/util/Point.cpp
#                                    src/util/Graph.cpp
#                                    src/util/BundleAdjuster.cpp
#                                    src/util/bal_problem.cpp
#                                    src/Viewer.cpp
#                                    src/Reader.cpp
#                                    src/Writer.cpp)
target_link_libraries(test_bundleAdjuster ${PROJECT_NAME})


