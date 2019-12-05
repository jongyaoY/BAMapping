# Config file for the Open3D package, defines the following variables
#  Open3D_INCLUDE_DIRS
#  Open3D_LIBRARIES
#  Open3D_LIBRARY_DIRS


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Open3DConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

####################################################################################

set(Open3D_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/Eigen;/usr/include/libdrm;/usr/include;${PACKAGE_PREFIX_DIR}/include/Open3D/3rdparty/fmt/include")
set(Open3D_LIBRARY_DIRS "${PACKAGE_PREFIX_DIR}/lib")
set(Open3D_LIBRARIES    "Open3D;GLEW;GLU;GL;glfw;/usr/lib/x86_64-linux-gnu/libGL.so;turbojpeg;jsoncpp;png16;z;tinyfiledialogs;tinyobjloader;qhullcpp;qhullstatic_r")

set(Open3D_C_FLAGS          "-fopenmp")
set(Open3D_CXX_FLAGS        "-fopenmp")
set(Open3D_EXE_LINKER_FLAGS "")
