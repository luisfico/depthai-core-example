# CMake C++ example project for depthai-core library

CMake example project which serves as a template on how to quickly get started with C++ and depthai library

## Depthai library dependencies
- cmake >= 3.2
- libusb1 development package
- C/C++11 compiler
 
MacOS: `brew install libusb`

Linux: `sudo apt install libusb-1.0-0-dev`

## Submodules
Make sure submodules are initialized and updated 
```
git submodule update --init --recursive
```

## Prerequisites 

### OpenCV - optional (for this example only) 

OpenCV is required to run the out-of-box example.
If OpenVINO is installed then no further action is required.

Otherwise install using one of the options below:

- Ubuntu 20.04 - `sudo apt install libopencv-dev`
 - MacOS - `brew install opencv`
 - Or install from package / sources (Windows and other OSes)
https://docs.opencv.org/master/d0/d3d/tutorial_general_install.html


## Building

Configure and build
```
cmake -S . -B build -D 'CMAKE_BUILD_TYPE=Release'
or
cmake -S . -B build -D 'CMAKE_BUILD_TYPE=Debug' 
cmake --build build --parallel -j$(nproc)
```

## Running

To run the example application 'myapp', navigate to build directory and run 'myapp' executable
```

./build/myapp Q4k_right.xml 

./build/rgb_depth_aligned_ref 

```

The results are generated in :

/home/lc/env/oakd/codeCpp/depthai-core-example/tmp




## Px Summary

C++ project
git remote add origin git@github.com:luisfico/depthai-core.git	     
(oak c++ running as example of library) OK get cloud from aligned 4Kimages with depth 400p. Min distance of deteccion=65cm   (Depth aligment with color camera performed by ISP oakd)

C++ project
git remote add origin git@github.com:luisfico/depthai-core-example.git  
(oak c++ isolated project ) ko get correct cloud  

Python project
git remote add origin git@github.com:luisfico/depthai-experiments.git   
(oak python get color ) OK get cloud from aligned stereo images 400p. Min distance of deteccion=35cm (no depth aligment with color camera. TODO)
