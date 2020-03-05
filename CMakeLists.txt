project(BAMapping)
cmake_minimum_required(VERSION 2.8)

IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)


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



#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(Ceres REQUIRED PATHS libs/ceres_lib)
include_directories(${CERES_INCLUDE_DIRS})

#include_directories(${GFLAGS_INCLUDE_DIRS})
#add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

find_package(Pangolin 0.4 REQUIRED PATHS libs/pangolin_lib)
include_directories(${Pangolin_INCLUDE_DIRS})

#find_package(CPUTSDF REQUIRED PATHS third_party/cpu_tsdf/build)
#include_directories(${CPUTSDF_INCLUDE_DIRS})
#link_directories(${CPUTSDF_LIBRARY_DIRS})

find_package(DBoW2 REQUIRED PATHS libs/dbow_lib/lib/cmake)
include_directories(${DBoW2_INCLUDE_DIRS})

find_package( OpenCV 3.4.8 EXACT REQUIRED PATHS libs/opencv_lib/share NO_DEFAULT_PATH)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(Open3D REQUIRED PATHS libs/open3d_lib/lib/cmake)
list(APPEND Open3D_LIBRARIES dl)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

include_directories(${Open3D_INCLUDE_DIRS})
add_definitions(-DUNIX)
add_compile_options(-Wno-deprecated-declarations)
add_compile_options(-Wno-unused-result)
add_definitions(-O3)
link_directories(${Open3D_LIBRARY_DIRS})


include_directories(${PROJECT_SOURCE_DIR}/libs/orbslam_lib/include)

add_library(${PROJECT_NAME} SHARED
            src/util/Parser.cpp
            src/Viewer.cpp
#            src/Reader.cpp
#            src/Writer.cpp
            src/Integrater.cpp
#new
            src/Frontend.cpp
            src/Graph.cpp
            src/Graph.h
            src/Frame.cpp
            src/Point.cpp
            src/BundleAdjuster.cpp
            src/Error.h
            src/GeometryMethods.cpp
            src/io/Reader.cpp
            src/io/Writer.cpp
            src/util/Converter.cpp
            src/util/Converter.h)
#add_library(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
                                 ceres
#                                 ${CHOLMOD_LIBRARIES}
#                                 ${GFLAGS_LIBRARIES}
                                 ${DBoW2_LIBRARIES}
                                 ${Pangolin_LIBRARIES}
                                 ${Open3D_LIBRARIES}
                                 ${OpenCV_LIBS}
                        )

#add_executable(System src/System.cpp)
#
#target_link_libraries(System ${PROJECT_NAME})

#add_executable(test_bundleAdjuster src/tests/test_bundleAdjuster.cpp)
#
#target_link_libraries(test_bundleAdjuster ${PROJECT_NAME}
#                                        ${Open3D_LIBRARIES})


#
#add_executable(test_orbslam src/tests/test_orbslam.cpp)
#target_link_libraries(test_orbslam  ${PROJECT_SOURCE_DIR}/libs/orbslam_lib/lib/libORB_SLAM2.so
#                                    ${OpenCV_LIBS}
#                                    ${Pangolin_LIBRARIES}
#                                    )




add_executable(test_Submap src/tests/test_Submap.cpp)

target_link_libraries(test_Submap ${PROJECT_NAME}
                                  ${Open3D_LIBRARIES})

add_executable(test_correspondence src/tests/test_correspondence.cpp)

target_link_libraries(test_correspondence ${PROJECT_NAME}
                                    ${Open3D_LIBRARIES})

add_executable(test_loopClosure src/tests/test_loopClosure.cpp)

target_link_libraries(test_loopClosure ${PROJECT_NAME}
        ${Open3D_LIBRARIES})

add_executable(test_json src/tests/test_json.cpp)
target_link_libraries(test_json ${PROJECT_NAME}
                                    )



