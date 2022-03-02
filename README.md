# Dense3DReconstruction
## Project Brief
This is official repository for Project for [Real-Time Globally Consistent Dense 3D Reconstruction With Online Texturing](https://ieeexplore.ieee.org/document/9184935)

This is a project from LuVision SIGMA, Tsinghua University. Visit our website for more interesting works: http://www.luvision.net/

Related papers:

> 1.  Multi-Index Hashing for Loop closure Detection. International Conference on Multimedia Expo, 2017. Best Student Paper Awards. 

> 2.  Beyond SIFT Using Binary features in Loop Closure Detection. IROS 2017. 

> 3.  FlashFusion: Real-time Globally Consistent Dense 3D Reconstruction using CPU Computing. RSS 2018.

> 4.  Real-Time Globally Consistent Dense 3D Reconstruction With Online Texturing. TPAMI 2020.

Please include citation below if you find our project helpful. 
```
@article{Dense3DReconstruction,
  author={Han, Lei and Gu, Siyuan and Zhong, Dawei and Quan, Shuxue and Fang, Lu},
  journal={IEEE Transactions on Pattern Analysis and Machine Intelligence}, 
  title={Real-Time Globally Consistent Dense 3D Reconstruction With Online Texturing}, 
  year={2022},
  volume={44},
  number={3},
  pages={1519-1533},
  doi={10.1109/TPAMI.2020.3021023}}
```

## License
This project is released under the [GPLv3 license](LICENSE). We only allow free use for academic use. For commercial use, please contact us to negotiate a different license by: `fanglu at tsinghua.edu.cn`

## Setup Instruction  ################################################################

### 0. Set up mirror address to accelerate installation

### 1. Install basic build tools:
```
sudo apt-get install build-essential pkg-config cmake git
```
### 2. Install required library step by step:

* First-to-install
```
sudo apt-get install ffmpeg libsuitesparse-dev libxmu-dev libgtk-3-dev libv4l-dev libglew-dev qtbase5-dev
```
* Glut
```
sudo apt-get install freeglut3-dev
```
* boost
```
sudo apt-get install libboost-all-dev
```
* Eigen3
```
sudo apt-get install libeigen3-dev
```
* Sophus
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus && mkdir build && cd build
cmake .. && make -j3 && sudo make install
```
* Pangolin
```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. && make -j3 && sudo make install
```
* OpenNI2
```
sudo apt-get install libopenni2-dev
```
* librealsense (to use realsense, check Intel's website to install essential driver)
```
sudo apt-get install libfreenect-dev libglfw3-dev
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense && mkdir build && cd build
cmake .. && make -j3 && sudo make install
```
* OpenCV 

```
sudo apt-get install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
git clone https://github.com/opencv/opencv.git
cd opencv && git checkout 8f1356c && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_TBB=ON -DWITH_LIBV4L=ON
-DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_CUDA=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF ..
make -j3 && sudo make install
sudo /bin/bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf' 
sudo ldconfig
```

### 3. Compile this repository
```
git clone https://github.com/THU-luvision/Dense3DReconstruction.git
cd Dense3DReconstruction && mkdir build && cd build
cmake .. &&make -j3
```

## Usage Instruction ##############################################################
```
./FlashFusion $DataFolder $ParametersFile Resolution inputID
```
For each input parameter:

* $DataFolder: base folder containing test data or output data. 

The basefolder should contain:
1.  calib.txt for calibration parameters
2.  associate.txt for input rgbd images
3.  groundtruth.txt for TUM RGBD dataset groundtruth data.

An example datafolder is provided [here](http://153.35.185.228:81/opensource_data/TextureFusion/synthesis.zip) 
and [here](http://153.35.185.228:81/opensource_data/TextureFusion/xtion.zip)
* $ParametersFile: input parameters, an example is given as settings.yaml

* Resolution: Reconstruction voxel resolution, ranges from 0.005 to 0.04

* InputMode:

  0: offline data stored in DataFolder
  1: Online data using ASUS Xtion
  2: Online data using Intel Realsense
  
### Example
```
cd $SourceDirectory/build
./FlashFusion  ~/Downloads/xtion/ ../settings.yaml 0.005 0
```
