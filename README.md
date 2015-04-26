# YAS

## Overview

1. [Introduction](#markdown-header-introduction)
2. [Building YAS](#markdown-header-build-instructions)
3. [Basic usage](#markdown-header-basic-usage)

## Introduction

### Theory

Details on the underlying theory can be found in

> J. Balzer, M. Peters, S. Soatto: Volumetric Reconstruction Applied to Perceptual Studies of Size and Weight, IEEE WACV, 2014.

A preprint is available [here](http://arxiv.org/abs/1311.2642
). Please cite if you use the software in your own published work. 

### Features

- acquisition and wide-baseline alignment of RGBD images 
- elaborate sensor model 
- Poisson mesh reconstruction
- ground plane estimation
- supports all Primesense/ONI devices 
- acquisition, import, and export of raw ONI data streams
- QT frontend
- file I/O of 
    - raw single-view RGB and disparity data (OpenEXR,PNG,PGM)
    - estimated poses (TXT or embedded in OpenEXR)
	- oriented point clouds (PLY) 
    - normal maps (R4R)
	- reconstructed triangular mesh (PLY)
- 3d viewer

### Contributors
- [J. Balzer](https://sites.google.com/site/jonabalzer/)
- [T. Mörwald](http://users.acin.tuwien.ac.at/tmoerwald/)

## Build instructions 

### Dependencies
- [OpenNI](http://structure.io/openni)
- [OpenCV](http://opencv.org/)
- [QT](http://qt-project.org/)
- [OpenEXR](http://www.openexr.com/)
- [libANN](http://www.cs.umd.edu/~mount/ANN/)
- [Boost](http://www.boost.org/)
- [OpenMP](http://openmp.org/wp/) (optional)

### Installing the driver

Unless you have upgraded your Ubuntu to version 13.04 or 13.10, you should be fine with the precompiled binaries which can be download from [Occipital](http://structure.io/openni). Simply unzip the driver and call the install script, while the sensor is not connected to your machine. Once the driver is installed, a good test to verify whether it is working properly is to plug in your device and run the `SimpleViewer` located in the subfolder `/Samples/Bin`.

Otherwise, you will need to download the sources from [GitHub](https://github.com/OpenNI/OpenNI2). Inside the make file `ThirdParty/PSCommon/BuildSystem/CommonCppMakefile` add `pthread` to the variable `$USED_LIBS` in line 29. From inside the root directory call 
```
sudo make
```
Create a folder 
```
mkdir Redist
```
and copy the contents of `/Bin/x64-Release` to it. Run 
```
sudo ./install.sh
```

### Building YAS

Clone the repository, e.g., by typing
```
git clone https://bitbucket.org/jbalzer/yas.git
```
This will automatically create a subfolder `yas`. Change into this directory
```
cd yas
```
and create a directory for the out-of-core build, say
```
mkdir build
```
Call qmake from the build directory specifying the location of the driver package (which you extracted previously from the archive in `/Packaging/Final`), e.g.,
```
cd build
qmake .. "OPENNI_DIR=/home/jbalzer/Workspace/OpenNI"
```
Start the build process by
```
make
```
**Important**: Any attempts to install the driver binaries in a system path failed so far (please, contact me if you are more successful). Consequently, you have to copy the contents of `$OPENNI_DIR/Redist` to the folder that contains the YAS executable, e.g., from within the build directory type
```
cp -ra /home/jbalzer/Workspace/OpenNI/Redist/. . 
```
Start YAS
```
./yas
```
and enjoy!

### A note for Mac users

The build process on MacOS is slightly more involved, the reason being that OpenEXR relies on POSIX semaphores for thread synchronization. Darwin, the basis of MacOS, is not 100% POSIX compliant in that it does not support semaphores. Any attempt to link against OpenEXR binaries included as part of OpenCV or obtained e.g. through MacPorts will cause a runtime error on EXR export. If you are working on MacOS you will have to build the OpenEXR library from source with semaphore support deactivated.

## Basic usage

The basic reconstruction workflow is as follows:

1. Start the live stream by clicking **Run** in the main window or load previous raw measurements stored in EXR format (see below).
2. Click on **Store** to add an RGBD image to the sparse set used for reconstruction. Make sure that there is enough overlap between two views. In our experience, roughly eight images taken from all around the object are sufficient for a complete reconstruction. Yet, the common wisdom applies that the smaller the baseline (in terms of translation **and** rotation), the likely it is that the alignment will be successful. Do not worry about holes. They will be filled in during Poisson reconstruction. The spin box in the lower left lets you iterate through all acquired images, the respective depth and RGB images being displayed in the lower half of the main window.
 
    **Hint**: Save and re-load raw data to EXR format with **File:Save all...** respectively **File:Open**.
  
3. Pairwise alignment is triggered (e.g., for parameter tuning) by selecting **Align:Current** from the **Tools** menu. This affects the view which is currently on display, say view number *i*, and the next one, i.e., view number *i+1*. If the last acquired image is selected, there will be no effect. The correspondence estimation and ICP refinement may take a little bit of time so be patient. A batch alignment is requested by selecting **Align:All**. 
3. You can verify the success of the last step by opening the OpenGL window (**View:3D View...**) and by iterating through the images with the spin box. That is because all point clouds are rendered w.r.t. the world (i.e., the first view's) reference frame. Additionally, upon completion of the alignment, the percentage of inlying matches is displayed in the status bar at the bottom of the main window. If the percentage drops below 10%, the alignment has probably failed. In that case, adjust the SIFT parameters (**View:Preferences**) or enrich the scene with textured objects, e.g., place the object on a newspaper or similar. If necessary, you can selectively remove views by hitting *Clear*. This affects the currently selected image. 
4. Unwanted clutter in fore- and background can be removed by operating the **Max/Min** sliders at the bottom of the main window. Inspect the cropping result in the bottom depth visualization. 
5. **Tools:Poisson** creates a triangle mesh from the point cloud. **File:Save** with the combo box set to *PLY* now saves this mesh instead of the oriented point cloud currently on display. 
	**Caveat**: At the present time, the third-party open-source code for Poisson reconstruction does not perform as well as the one integrated in Meshlab. We will investigate the issue and fix it in future releases. For the time being, it is recommended to export the aligned oriented point clouds to PLY (choose **Save all...** from the **File** menu, enter a prefix, select *PLY* from the combo box in the lower right of the dialogue and hit **OK**), import them to Meshlab, merge visible layers (right-click on the layers panel, don't forget to check *Keep unreferenced vertices*), and finally start mesh reconstruction with **Filter:Point Set:Surface Reconstruction:Poisson**. 

See this [short video clip](http://youtu.be/T1rRg-LkLio) to familiarize yourself with the basic workflow of building models with YAS from sparse view points. 
