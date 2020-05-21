project(BAMapping)
cmake_minimum_required(VERSION 2.8)

IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)


find_package(Ceres REQUIRED PATHS libs/ceres_lib)
include_directories(${CERES_INCLUDE_DIRS})


find_package(Pangolin 0.4 REQUIRED PATHS libs/pangolin_lib)
include_directories(${Pangolin_INCLUDE_DIRS})

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


include_directories(${PROJECT_SOURCE_DIR}/src)

add_library(${PROJECT_NAME} SHARED
            src/util/Parser.cpp
            src/Viewer.cpp
            src/Integrater.cpp
#new
            src/frontend/System.cpp
            src/frontend/Tracking.cpp
            src/frontend/Mapping.cpp
            src/frontend/LoopClosing.cpp
            src/frontend/Alignment3D_ransac.cpp

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

target_link_libraries(${PROJECT_NAME}
                                 ceres
                                 ${DBoW2_LIBRARIES}
                                 ${Pangolin_LIBRARIES}
                                 ${Open3D_LIBRARIES}
                                 ${OpenCV_LIBS}
                        )



##TUM
add_executable(runFrontendTUM src/examples/runFrontendTUM.cpp)

target_link_libraries(runFrontendTUM ${PROJECT_NAME})

add_executable(runBackendTUM src/examples/runBackendTUM.cpp)

target_link_libraries(runBackendTUM ${PROJECT_NAME}
                                    ${Open3D_LIBRARIES})
##ITE

add_executable(runFrontendITE src/examples/runFrontendITE.cpp)

target_link_libraries(runFrontendITE ${PROJECT_NAME})

add_executable(runBackendITE src/examples/runBackendITE.cpp)

target_link_libraries(runBackendITE ${PROJECT_NAME})

add_executable(runFrontend src/examples/runFrontend.cpp)
target_link_libraries(runFrontend ${PROJECT_NAME})

add_executable(localOptimization src/examples/localOptimization.cpp)

target_link_libraries(localOptimization ${PROJECT_NAME}
        ${Open3D_LIBRARIES})

add_executable(globalOptimization src/examples/globalOptimization.cpp)

target_link_libraries(globalOptimization ${PROJECT_NAME}
                                        ${Open3D_LIBRARIES})

##tools

add_executable(Visualizer src/Visualizer.cpp)
target_link_libraries(Visualizer ${Open3D_LIBRARIES})

add_executable(integrateGraph src/examples/integrateGraph.cpp)

target_link_libraries(integrateGraph ${PROJECT_NAME}
                                 ${Open3D_LIBRARIES})

add_executable(createVocabulary src/examples/createVocabulary.cpp)
target_link_libraries(createVocabulary ${PROJECT_NAME})







#add_executable(test_Submap src/tests/test_Submap.cpp)
#
#target_link_libraries(test_Submap ${PROJECT_NAME}
#                                  ${Open3D_LIBRARIES})
#
#add_executable(test_correspondence src/tests/test_correspondence.cpp)
#
#target_link_libraries(test_correspondence ${PROJECT_NAME}
#                                    ${Open3D_LIBRARIES})
#
#
#
#add_executable(test_loopClosure src/tests/test_loopClosure.cpp)
#
#target_link_libraries(test_loopClosure ${PROJECT_NAME}
#        ${Open3D_LIBRARIES})
#
#add_executable(test_json src/tests/test_json.cpp)
#target_link_libraries(test_json ${PROJECT_NAME})





