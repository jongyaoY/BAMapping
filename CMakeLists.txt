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

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include_directories(${GFLAGS_INCLUDE_DIRS})
#add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(CPUTSDF REQUIRED PATHS third_party/cpu_tsdf/build)
include_directories(${CPUTSDF_INCLUDE_DIRS})
link_directories(${CPUTSDF_LIBRARY_DIRS})


find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(${PROJECT_NAME} SHARED
            src/util/Optimize.cpp
            src/util/Frame.cpp
            src/util/Point.cpp
            src/util/Graph.cpp
            src/util/BundleAdjuster.cpp
            src/util/bal_problem.cpp
            src/Viewer.cpp
            src/Reader.cpp
            src/Writer.cpp
            src/Integrater.cpp)
#add_library(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
                                 ceres
                                 ${CPUTSDF_LIBRARIES}
#                                 ${CHOLMOD_LIBRARIES}
#                                 ${GFLAGS_LIBRARIES}
                                 ${Pangolin_LIBRARIES}
                                 ${PCL_LIBRARIES}
                                 ${OpenCV_LIBS})

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

add_executable(test_cpu_tsdf src/tests/test_cpu_tsdf.cpp)
target_link_libraries(test_cpu_tsdf ${CPUTSDF_LIBRARIES}
                                    ${PCL_LIBRARIES}
                                    ${OpenCV_LIBS}
                                    ${PROJECT_NAME})

