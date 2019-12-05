#pangolin:
#dependency:
#OpenGL
sudo apt install libgl1-mesa-dev
#Glew
sudo apt install libglew-dev
#CMake
sudo apt install cmake
#Wayland

#pkg-config: 
sudo apt install pkg-config
#Wayland and EGL:
sudo apt install libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols

cd Pangolin
mkdir build
cd build
cmake ..
cmake -DCMAKE_INSTALL_PREFIX="../libs/pangolin_lib" --build .
make install
