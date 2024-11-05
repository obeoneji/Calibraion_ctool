# Calibraion_ctool
Calibraion_ctool is a calibration tool for underwater camera system based on C++

Features include:
- Camera housing calibration based on '[Refractive Geometry for Underwater Domes](https://doi.org/10.1016/j.isprsjprs.2021.11.006)'
	- with a Flat Port model from '[Refractive Calibration of Underwater Cameras](https://doi.org/10.1007/978-3-642-33715-4_61)'
- Camera and stereo camera calibration 
- Calibration guidance implementation of '[Calibration Wizard](https://doi.org/10.1109/iccv.2019.00158)'
- [COLMAP](https://colmap.github.io/) compliant camera models
- [G2O](https://github.com/RainerKuemmerle/g2o/)  bundle adjustment solutions
- [OpenCV](https://github.com/opencv/opencv/) Feature points extractor

## Build from Source

### CUDA

To build with CUDA support install the latest CUDA from NVIDIA's homepage.

You will also need to specify a CUDA architecture during cmake configuration as e.g. `-DCMAKE_CUDA_ARCHITECTURES=native`.

There is a known error for Windows, when installing CUDA together with Visual Studio Build Tools (as opposed to Visual Studio IDE), where CUDA does not properly integrate into the build tools and some files have to be manually copied.

### Windows

For Windows it is recommended to use [vcpkg](https://github.com/microsoft/vcpkg).
Please use latest release. You will also need Visual Studio Build Tools.

    git clone https://github.com/microsoft/vcpkg
    cd vcpkg
    bootstrap-vcpkg.sh
    vcpkg install colmap[cuda]:x64-windows
    vcpkg install g2o:x64-windows

This will take a while.
Then you should complie OpenCV and OpenCV contrib. Download v4.9.0 in opencv( https://opencv.org/releases/) and opencv-contrib(https://github.com/opencv/opencv_contrib/releases/tag/4.9.0).
Extract opencv_contrib and Place files in the opencv folder `\path\to\your\opencvdir\opencv_contrib-4.9.0`, compile and install OpenCV:

    //opencv
    mkdir build
    mkdir install
    cd  build
    cmake .. -DCMAKE_TOOLCHAIN_FILE=C:\Users\Mayn\work\3rdparty\vcpkg-2024.10.21/scripts/buildsystems/vcpkg.cmake -DCMAKE_INSTALL_PREFIX=C:\Users\Mayn\work\3rdparty\opencv\install -DBUILD_SHARED_LIBS=ON -DOPENCV_EXTRA_MODULES_PATH=C:\Users\Mayn\work\3rdparty\opencv\opencv_contrib-4.9.0\modules
    cmake --build . --config Release
    cmake --install .
Then, add `\path\to\your\opencvdir\opencv-4.9.0\install\x64\vc16\bin` to your system environment variables.
Configure and compile Calibraion_ctool:

	cd path/to/calibmar
    mkdir build
    cd build
    cmake .. -DCMAKE_TOOLCHAIN_FILE=path/to/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_CUDA_ARCHITECTURES=native
    cmake --build . --config Release

CMake Presets are available which are supported by several IDEs. Presets for windows expect the environment variable `VCPKG_ROOT` to point to the vcpkg root directory.
