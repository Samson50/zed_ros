# Stereolabs ZED - PCL

This sample shows how to interface the ZED SDK and PCL (Point Cloud Library) to acquire and display a 3D point cloud from the ZED.


## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com).
- Make sure you have [PCL](https://github.com/PointCloudLibrary/pcl) installed with its visualization module.
- For more information, read the ZED [API documentation](https://www.stereolabs.com/developers/documentation/API/).


### Prerequisites

- Windows 7 64bits or later, Ubuntu 16.04
- [ZED SDK](https://www.stereolabs.com/developers/) and its dependencies ([CUDA](https://developer.nvidia.com/cuda-downloads), [OpenCV 3.1](http://opencv.org/downloads.html))


## Build the program

Download the sample and follow these instructions:

#### Build for Windows

  - Create a folder called "build" in the source folder
  - Open cmake-gui and select the source and build folders
  - Generate the Visual Studio `Win64` solution
  - Open the resulting solution and change the solution configuration to `Release`
  - Build solution

#### Build for Linux

Open a terminal in the sample directory and execute the following command:

    mkdir build
    cd build
    cmake ..
    make

## Run the program

- Navigate to the build directory and launch the executable file
- Or open a terminal in the build directory and run the sample :

        ./ZED\ with\ PCL [path to SVO file]

You can optionally provide an SVO file path (recorded stereo video of the ZED)


## Features

The ZED SDK outputs a 4-channel float matrix containing the XYZRGB point cloud. This color point cloud is converted to PCL format and then displayed using `PCLVisualizer`.



### Troubleshooting
 - PCL view can be zoomed in at launch, displaying only green and red colors. To adjust the viewport, simply zoom out using your mouse wheel or keyboard shortcuts.

### Limitations

- The point cloud conversion to PCL format is time consuming and will affect application running time.
