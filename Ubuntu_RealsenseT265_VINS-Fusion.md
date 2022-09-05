# Ubuntu20.04+RealsenseT265运行VINS-Fusion

# 基本定义
1. 相机内参数是与相机自身特性相关的参数，比如相机的焦距、像素大小等；
1. 相机外参数是在世界坐标系中的参数，比如相机的位置、旋转方向等。

1.外参数矩阵。告诉你现实世界点(世界坐标)是怎样经过旋转和平移，然后落到另一个现实世界点(摄像机坐标)上。

2.内参数矩阵。告诉你上述那个点在1的基础上，是如何继续经过摄像机的镜头、并通过针孔成像和电子转化而成为像素点的。

3.畸变矩阵。告诉你为什么上面那个像素点并没有落在理论计算该落在的位置上，还tm产生了一定的偏移和变形！！！

2、[摄像机内参、外参矩阵](http://blog.csdn.net/liyuan123zhouhui/article/details/52043683)  
在opencv的3D重建中（opencv中文网站中：照相机定标与三维场景重建），对摄像机的内参外参有讲解：   
外参：摄像机的旋转平移属于外参，用于描述相机在静态场景下相机的运动，或者在相机固定时，运动物体的刚性运动。因此，在图像拼接或者三维重建中，就需要使用外参来求几幅图像之间的相对运动，从而将其注册到同一个坐标系下面来   
内参：下面给出了内参矩阵，需要注意的是，真实的镜头还会有径向和切向畸变，而这些畸变是属于相机的内参的。   
摄像机内参矩阵：

```markdown
        fx   s    x0    
   K =  0    fy   y0
        0    0    1
```

其中，fx，fy为焦距，一般情况下，二者相等，x0、y0为主点坐标（相对于成像平面），s为坐标轴倾斜参数，理想情况下为0

摄像机外参矩阵：包括旋转矩阵和平移矩阵   
旋转矩阵和平移矩阵共同描述了如何把点从世界坐标系转换到摄像机坐标系

[旋转矩阵](http://www.cnblogs.com/caster99/p/4703033.html%20%E6%97%8B%E8%BD%AC%E7%9F%A9%E9%98%B5)：描述了世界坐标系的坐标轴相对于摄像机坐标轴的方向   
平移矩阵：描述了在摄像机坐标系下，空间原点的位置

https://blog.csdn.net/liulina603/article/details/52953414 


# 一、提前条件

系统版本：Ubuntu20.04+ROS（noetic）  
默认已经掌握了ubuntu系统下的基本命令以及ROS的基本操作

# 二、realsenseT265的SDK测试

官方网站[https://www.intelrealsense.com/get-started-tracking-camera/](https://www.intelrealsense.com/get-started-tracking-camera/)  
照着其中[https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)安装过程进行安装即可，保证运行realsense-viewer后能有相关界面。

# 三、realsenseT265的标定

## 1.准备工作（需要注意以下文件编译过程中，可能出现依赖库缺失的报错，这很正常，按照提示的错误信息安装对应依赖库即可）

**1.1 下载并编译ceres**

```bash
git clone https://github.com/ceres-solver/ceres-solver
cd ceres
mkdir build
cd build
cmake ..
make
sudo make install
```

**1.2 下载并编译code_utils**

首先，安装依赖库

```bash
sudo apt-get install libdw-dev
```

之后，安装code_utils

```bash
cd ~/catkin_ws/src
git clone https://github.com/gaowenliang/code_utils
cd ..
catkin build
source ~/catkin_ws/devel/setup.bash
```

- 如果有报错`fatal error: backward.hpp: 没有那个文件或目录`。此时在`code_utils`下面找到`sumpixel_test.cpp`，修改`#include "backward.hpp"`为`#include “code_utils/backward.hpp”`后再catkin build。  

- 我也遇见过`no module named “XXX”`的错误，是因为没安装对应的依赖库，修改 package.xml添加 <depend>code_utils</depend>，安装后再catkin build即可。

- fatal error: numpy/arrayobject.h: No such file or directory

```
 	sudo apt-get install python-numpy
```

 https://blog.csdn.net/qq_39607707/article/details/125061020 
 **问题1：**
- CMakeLists.txt文件下代码修改如下：

```cpp
修改set(CMAKE_CXX_FLAGS "-std=c++11")为set(CMAKE_CXX_FLAGS "-std=c++14")
```

**问题2：**  
![在这里插入图片描述](https://img-blog.csdnimg.cn/b904d1694ab24f02a2b01798db0f1578.png#pic_center)  
修改：

```cpp
添加头文件：#include"opencv2/imgcodecs/legacy/constants_c.h"
```

**问题3：**  
![在这里插入图片描述](https://img-blog.csdnimg.cn/351be26796c74d9c96b9fabd8500812d.png#pic_center)  
修改：

```cpp
CV_MINMAX 改为 NORM_MINMAX
```

#### 2.1.3 mat_io_test.cpp文件下

代码修改如下：

**问题：**  
![在这里插入图片描述](https://img-blog.csdnimg.cn/75c6d1e234744405ae576418d23c6ad0.png#pic_center)

> opencv4.x以上，有些宏，API名字改了，需要改为新的：  
> CV_LOAD_IMAGE_UNCHANGED 改为 cv::IMREAD_UNCHANGED  
> CV_LOAD_IMAGE_GRAYSCALE 改为 cv::IMREAD_GRAYSCALE  
> CV_LOAD_IMAGE_COLOR 改为 cv::IMREAD_COLOR  
> CV_LOAD_IMAGE_ANYDEPTH 改为 cv::IMREAD_ANYDEPTH

修改：

```cpp
CV_LOAD_IMAGE_UNCHANGED 改为 cv::IMREAD_UNCHANGED
```


**1.3 下载并编译imu_utils**

```bash
cd ~/catkin_ws/src
git clone https://github.com/gaowenliang/imu_utils.git
cd ..
catkin build
source ~/catkin_ws/devel/setup.bash
```

注意：先编译code_utils，再编译imu_utils，不能放在一起编译。

#### 2.2.1 CMakeLists.txt文件下

代码修改如下：

```cpp
修改set(CMAKE_CXX_FLAGS "-std=c++11")为set(CMAKE_CXX_FLAGS "-std=c++14")
```

#### 2.2.2 imu_an.cpp文件下

代码修改如下：

**问题：**  
![在这里插入图片描述](https://img-blog.csdnimg.cn/5ccfd0cc1ef14550b19a9b0c55e3b170.png#pic_center)


修改：

```cpp
添加头文件：#include <fstream>
```


**1.4 下载并编译kalibr**

首先安装依赖库

```bash
sudo apt-get install python-setuptools
sudo apt-get install python-setuptools python-rosinstall ipython libeigen3-dev libboost-all-dev doxygen
sudo apt-get install ros-noetic-vision-opencv ros-noetic-image-transport-plugins ros-noetic-cmake-modules python-software-properties software-properties-common libpoco-dev python-matplotlib python-scipy python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev python-catkin-tools libv4l-dev
```

接下来安装kalibr

```bash
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/Kalibr.git
cd ..
catkin build
source ~/catkin_ws/devel/setup.bash
```

在看其他博客时发现可能会出现python相关的问题，我用的是自带的Python 2.7.12版本，在安装过程中没有出现问题。

**1.5 下载并安装realsense-ros**  
首先安装依赖库

```bash
sudo apt-get install ros-noetic-ddynamic-reconfigure
```

接下来安装realsense-ros

```bash
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros
cd ~/catkin_ws
catkin build
source ~/catkin_ws/devel/setup.bash
```

至此，我们的准备工作就做好了。

## 2.标定流程

##### 2.0 准备标定板

标定板可以用 kalibr 提供的pdf　下载地址为：https://github.com/ethz-asl/kalibr/wiki/downloads, 由于Aprilgrid能提供序号信息, 能够防止姿态计算时出现跳跃的

情况,所以这里采用Aprilgrid 6x6 0.8x0.8 m (A0 page)进行标定。

我下载了这个pdf, 打印成了Ａ4纸大小，在标定前, 注意测量格子的尺寸信息填入yaml文件, 尺寸信息具体是哪些数据可以看看kalibr的说明，说明的网址为：　

https://github.com/ethz-asl/kalibr/wiki/calibration-targets



本篇使用了二维码标定板，创建或修改apriltag.yaml

```ruby
target_type: 'aprilgrid' #gridtypetagCols: 6               #number of apriltagstagRows: 6               #number of apriltagstagSize: 0.024           #size of apriltag, edge to edge [m]tagSpacing: 0.3          #ratio of space between tags to tagSize
```

其中tagsize和tagspacing是要根据实际打印出来的标定板做修改的，其参数意义可参照下图，该图同样来自[https://github.com/ethz-asl/kalibr/wiki/calibration-targets](https://github.com/ethz-asl/kalibr/wiki/calibration-targets "https://github.com/ethz-asl/kalibr/wiki/calibration-targets")。



**2.1修改工作空间下文件的读写权限，不然后面在运行时会报错**

```bash
 cd ~/catkin_ws
 sudo chmod 777 src/ -R
```

**2.2 相关文件的修改**  
打开位于`realsense-ros/realsense2_camera/launch`目录下的`rs_t265.launch`文件，将原本的代码`<arg name="unite_imu_method" default=""/>`修改为`<arg name="unite_imu_method" default="linear_interpolation"/>`。  
在`~/catkin_ws/src/imu_utils/launch`中新建`t265_imu.launch`如下

```bash
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "/camera/imu"/> 
        <param name="imu_name" type="string" value= "BMI055"/>
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <param name="max_time_min" type="int" value= "60"/> 
        <param name="max_cluster" type="int" value= "200"/> 
    </node>
</launch>
```

**2.3 IMU的校准**  
将realsenseT265插上电脑后，建议在你喜欢的地方新建一个文件夹，比如我是在桌面上新建了一个文件夹，然后在其中打开终端，输入以下命令

```bash
roslaunch realsense2_camera rs_t265.launch
roslaunch imu_utils t265_imu.launch
```

注意过程中显示`wait for imu data`是正常情况，等待大约60分钟即可出结果，在你新建的文件夹内生成了`BMI055_imu_param.yaml`文件，该文件给出了加速度计和陀螺仪三轴的noise_density(后缀n)和random_walk(后缀w)，同时计算出了平均值，后面IMU+摄像头联合标定的时候需要这些均值。



### 2.2.3 标定及结果

1）标定

这里是双目相机标定，单目相机也是同理。

```css
rosrun kalibr kalibr_calibrate_cameras --bag /home/luoteng/kalibr_ws/bag/camlr1.bag --topics /caml /camr --models pinhole-radtan pinhole-radtan --target /home/luoteng/kalibr_ws/doc/board/apriltag.yaml --show-extraction --bag-from-to 5 105
```

\--bag后面是录制的数据包的**绝对路径**；

\--topics后面是待标定的话题数据（已经控制频率后的）；

\--models后面是相机/畸变模型，有几目相机就要写几个，这里是两个针孔模型相机，所以是两个pinhole-radtan;相应地，其他支持的模型可以查看[https://github.com/ethz-asl/kalibr/wiki/supported-models](https://github.com/ethz-asl/kalibr/wiki/supported-models "https://github.com/ethz-asl/kalibr/wiki/supported-models")。

\--target后面是标定板参数文件的**绝对路径**；

\--show-extraction是在标定过程中的一个显示界面，可以看到图片提取的过程，可以不要；

\--bag-from-to后面是想要使用数据时间段的起始时间和结束时间，单位：秒(s)，这个参数可以剔除掉刚开始录制和结束时一些出入视野等画面。



### 个人遇到的问题记录：

标定运行过程中报错，RuntimeError: Optimization failed!

可以参照github上作者的回答[https://github.com/ethz-asl/kalibr/issues/41](https://github.com/ethz-asl/kalibr/issues/41 "https://github.com/ethz-asl/kalibr/issues/41")，提高timeOffsetPadding ([https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera#L171](https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera#L171 "https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera#L171"))再次尝试。

2）标定结果

https://blog.csdn.net/OptimusAlpha/article/details/122325372

标定完成会生成下面3个文件，其中命名为camchain的.yaml文件是后续联合标定要继续用到的，里面包含了所需的相机的内外参。

![](https://img-blog.csdnimg.cn/efee6ddc9bd0470f93e84fbf4feba68e.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAT3B0aW11c0FscGhh,size_11,color_FFFFFF,t_70,g_se,x_16)

可以查看report的pdf，重投影误差（reprojection errors）在1个像素以内标定就是比较好的了。

 ![](https://img-blog.csdnimg.cn/ebc8dd3448b140619d64d7d38db0ed51.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAT3B0aW11c0FscGhh,size_20,color_FFFFFF,t_70,g_se,x_16)![](https://img-blog.csdnimg.cn/8d1e6b063ac345ce8ba8385721a468a8.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAT3B0aW11c0FscGhh,size_20,color_FFFFFF,t_70,g_se,x_16)

命名为camchain的.yaml文件内容如下。

```markdown
cam0:  cam_overlaps: [1]  camera_model: pinhole  distortion_coeffs: [-0.11618573305493797, 0.06422069640456901, -0.00013436975723873972,    -0.001005182072042385]  distortion_model: radtan  intrinsics: [781.3078393884367, 777.6555008477854, 818.0060374225857, 637.1024477055486]  resolution: [1624, 1240]  rostopic: /camlcam1:  T_cn_cnm1:  - [-0.997500970365195, 0.038409235650546554, -0.0593004615263394, -0.04013241756555838]  - [-0.0384584418581223, -0.9992601518503074, -0.000311727361527353, 0.48583888428319955]  - [-0.05926856139929079, 0.001969655006157624, 0.9982401304740343, -0.007530598232056542]  - [0.0, 0.0, 0.0, 1.0]  cam_overlaps: [0]  camera_model: pinhole  distortion_coeffs: [-0.12076005804518206, 0.0672304142796621, 0.0002881412854617891,    0.000123605852697794]  distortion_model: radtan  intrinsics: [779.6463647028914, 776.406545014067, 795.0043893364035, 622.8861461860073]  resolution: [1624, 1240]  rostopic: /camr
```



**2.4 相机的标定**  
下载官方给的`april_6x6_80x80cm_A0.pdf`或者其它标定文件。打印或者在屏幕显示，测量实际的尺寸后，在你之前新建的文件夹中新建`apriltags.yaml`，我的文件内容如下：

```bash
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.16           #size of apriltag, edge to edge [m]
tagSpacing: 0.3125         #ratio of space between tags to tagSize
                         #example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]
```

之后，在你新建的文件夹中打开终端，开启realsenseT265

```bash
roslaunch realsense2_camera rs_t265.launch
```

修改话题发布频率

```bash
rosrun topic_tools throttle messages /camera/fisheye1/image_raw 10.0 /fisheye1
rosrun topic_tools throttle messages /camera/fisheye2/image_raw 10.0 /fisheye2
```

录制文件，注意录制过程中要缓慢移动相机，使其能看到完整清晰的标定文件（可以先在录制前打开rviz，调用image的话题进行观察，判断移动的位置）

```bash
rosbag record -O cameras_calibration /fisheye1 /fisheye2
```

调用kalibr的算法计算各个摄像头的内参和外参

```cpp
kalibr_calibrate_cameras --target ./apriltags.yaml --bag ./cameras_calibration.bag --bag-from-to 2 35 --models omni-radtan omni-radtan --topics /fisheye1 /fisheye2 
```

因为并不一定要用整个录制视频，2和35是你想要的起始和截止时间，可以修改。  
如果在过程中出现`Using the default setup in the initial run leads to an error of Cameras are not connected through mutual observations, please check the dataset. Maybe adjust the approx. sync. tolerance.`相机不同步的报错，可以通过修改话题发布的频率，或者在kalibr命令的末尾加上–approx-sync 0.04来解决。  
最终会生成`camchain-.cameras_calibration.yaml`、`results-cam-.cameras_calibration.txt和report-cam-.cameras_calibration.pdf`。  
**2.5. Camera-IMU联合标定**  
在你之前新建的文件夹中，新建`imu.yaml`文件如下（根据你之前的`BMI055_imu_param.yaml`填写参数）：

```bash
#Accelerometers
accelerometer_noise_density: 8.003e-02   #Noise density (continuous-time)
accelerometer_random_walk:   5.382e-03   #Bias random walk

#Gyroscopes
gyroscope_noise_density:     8.163e-03   #Noise density (continuous-time)
gyroscope_random_walk:       3.470e-05   #Bias random walk

rostopic:                    /imu      #the IMU ROS topic
update_rate:                 200.0      #Hz (for discretization of the values above)
```

修改`rs_t265.launch`其中的两行代码如下：

```bash
<arg name="enable_sync"         default="true"/> 
<arg name="unite_imu_method"    default="copy"/>
```

进入你之前新建的文件夹，打开终端，开启T265

```bash
roslaunch realsense2_camera rs_t265.launch
```

修改发布频率

```bash
rosrun topic_tools throttle messages /camera/fisheye1/image_raw 20.0 /fisheye1
rosrun topic_tools throttle messages /camera/fisheye2/image_raw 20.0 /fisheye2
rosrun topic_tools throttle messages /camera/imu 200.0 /imu
```

录制文件

```bash
rosbag record -O imu_cameras_calibration /fisheye1 /fisheye2 /imu
```

调用kalibr的算法计算IMU和camera外参

```bash
kalibr_calibrate_imu_camera --target ./apriltags.yaml --cam ./camchain-.cameras_calibration.yaml --imu ./imu.yaml --bag ./imu_cameras_calibration.bag --bag-from-to 2 35 --max-iter 30 --show-extraction
```

最终会输出`camchain-imucam-.imu_cameras_calibration.yaml`、`imu-.imu_cameras_calibration.yaml`、`results-imucam-.imu_cameras_calibration.txt`、`report-imucam-.imu_cameras_calibration.pdf`四个文件，你可以通过pdf文件查看你标定的准确性。  
至此，我们完成了相机和IMU的标定。

# 四、运行VINS-FUSION

先下载并编译VINS-Fusion

```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
cd ..
catkin build
source ~/catkin_ws/devel/setup.bash
```

## Ubuntu 20 编译报错
https://blog.csdn.net/xiaojinger_123/article/details/121517771

编译时会发现一堆错误，别急，无非就是环境冲突问题，一个个解决

1）首先，ROS noetic版本中自带的OpenCV4和VINS-mono中需要使用的OpenCV3冲突的问题。修改vins-mono代码兼容opencv4。其实主要修改的是camera_model这个包，幸运的是发现不用修改代码中不兼容的变量，而是可以直接包含缺失的头文件即可。于是参考opencv参考文档查找opencv4中未定义的变量在opencv3中所属的头文件，然后添加到camera_model相应的头文件中顺利解决问题。

```cpp
在camera_model包中的头文件Chessboard.h中添加
#include <opencv2/imgproc/types_c.h>
#include <opencv2/calib3d/calib3d_c.h>
在CameraCalibration.h中添加
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
```

2）报错：

```cpp
/usr/local/include/ceres/internal/integer_sequence_algorithm.h:64:21: error: ‘integer_sequence’ is not a member of ‘std’
   64 | struct SumImpl<std::integer_sequence<T, N, Ns...>> {
      |                     ^~~~~~~~~~~~~~~~
```

解决：  
这是因为较新版本中的ceres对c++版本有要求

在报错的项目的CMakeList里的  
set(CMAKE_CXX_FLAGS “-std=c++11”)  
改成  
set(CMAKE_CXX_STANDARD 14)

3）编译时遇到报错 error: ‘CV_FONT_HERSHEY_SIMPLEX’ was not declared in this scope  
将报错文件上的 CV_FONT_HERSHEY_SIMPLEX 参数改为 cv::FONT_HERSHEY_SIMPLEX

4）报错：

```cpp
error: ‘CV_RGB2GRAY’ was not declared in this scope
   53 |       cv::cvtColor(image, aux, CV_RGB2GRAY);
      |                                ^~~~~~~~~~~
```

解决：  
在报错头文件里添加

#include <opencv2/imgproc/types_c.h>

5）报错：

```cpp
error: ‘CV_LOAD_IMAGE_GRAYSCALE’ was not declared in this scope
  125 |    imLeft = cv::imread(leftImagePath,  CV_LOAD_IMAGE_GRAYSCALE );
      |                                        ^~~~~~~~~~~~~~~~~~~~~~~
```

但在Opencv4中，CV_LOAD_IMAGE_GRAYSCALE找不到，经过查看Opencv的API可知，CV_LOAD_IMAGE_GRAYSCALE已改为 IMREAD_GRAYSCALE，修改即可。

修改之后即可编译成功。

## 开始运行

在`VINS-Fusion/config`文件夹中，新建文件夹名为`realsense_t265`，并在其中新建`fisheye1.yaml`,`fisheye2.yaml`,`stereo_imu.yaml`三个文档，内容如下（注意相关参数需要参考`BMI055_imu_param.yaml`、`results-cam-.cameras_calibration.txt`和`results-imucam-.imu_cameras_calibration.txt`自行修改）：

`fisheye1.yaml`

```bash
%YAML:1.0
---
model_type: MEI
camera_name: camera
image_width: 848
image_height: 800
mirror_parameters:
   xi: 1.75347951
distortion_parameters:
   k1: 0.03509054
   k2: -0.3590536
   p1: 0.00250588
   p2: 0.00058101
projection_parameters:
   gamma1: 788.10829678
   gamma2: 788.19861186
   u0: 416.33019648
   v0: 404.21771229
```

`fisheye2.yaml`

```bash
%YAML:1.0
---
model_type: MEI
camera_name: camera
image_width: 848
image_height: 800
mirror_parameters:
   xi: 1.95209328
distortion_parameters:
   k1: 0.18993538
   k2: -0.8234989
   p1: 0.00337246
   p2: 0.00013959
projection_parameters:
   gamma1: 843.77832442
   gamma2: 843.19711184
   u0: 419.54774026
   v0: 406.36245572
```

`stereo_imu.yaml`

```bash
%YAML:1.0

#common parameters
#support: 1 imu 1 cam; 1 imu 2 cam: 2 cam; 
imu: 1         
num_of_cam: 2  

imu_topic: "/camera/imu"
image0_topic: "/camera/fisheye1/image_raw"
image1_topic: "/camera/fisheye2/image_raw"
output_path: "~/output"   

cam0_calib: "fisheye1.yaml"
cam1_calib: "fisheye2.yaml"
image_width: 848
image_height: 800
   

# Extrinsic parameter between IMU and Camera.
estimate_extrinsic: 1   # 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.
                        # 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.

body_T_cam0: !!opencv-matrix #cam0 coordinate under body coordinate
   rows: 4
   cols: 4
   dt: d
   data: [-0.9999306582373404, 0.001093290620038494, 0.01172533294445286, 0.5201790241280906,
                -0.0011878229478030982, -0.9999668259020238, -0.008058298336212454, 0.04796159845159734,
                0.011716133905124508, -0.008071667179143842, 0.9998987850754022, -0.05434762530417168,
                0., 0., 0., 1.]

body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [-0.9999985320918137, 0.0011840662046389947, -0.0012384673758261621, -0.10570692194161202,
                -0.001173065649638605, -0.999960187777609, -0.008845720815767944, 0.048863128512499784,
                -0.0012488919887611945, -0.008844255027525165, 0.9999601089152549, -0.04892047362600982,
                0., 0., 0., 1. ]

#Multiple thread support
multiple_thread: 1

#feature traker paprameters
max_cnt: 150            # max feature number in feature tracking
min_dist: 30            # min distance between two features 
freq: 10                # frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
F_threshold: 1.0        # ransac threshold (pixel)
show_track: 1           # publish tracking image as topic
flow_back: 1            # perform forward and backward optical flow to improve feature tracking accuracy

#optimization parameters
max_solver_time: 0.04  # max solver itration time (ms), to guarantee real time
max_num_iterations: 8   # max solver itrations, to guarantee real time
keyframe_parallax: 10.0 # keyframe selection threshold (pixel)

#imu parameters       The more accurate parameters you provide, the better performance
acc_n: 8.00274e-02          # accelerometer measurement noise standard deviation. #0.2   0.04
gyr_n: 8.16326e-03         # gyroscope measurement noise standard deviation.     #0.05  0.004
acc_w: 5.38231e-03         # accelerometer bias random work noise standard deviation.  #0.002
gyr_w: 3.470378e-05       # gyroscope bias random work noise standard deviation.     #4.0e-5
g_norm: 9.805         # gravity magnitude

#unsynchronization parameters
estimate_td: 0                      # online estimate time offset between camera and imu
td: 0.0                             # initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)

#loop closure parameters
load_previous_pose_graph: 0        # load and reuse previous pose graph; load from 'pose_graph_save_path'
pose_graph_save_path: "~/output" # save and load path
save_image: 1                   # save image in pose graph for visualization prupose; you can close this function by setting 0 
```

之后打开终端，运行

```bash
roslaunch realsense2_camera rs_t265.launch
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/realsense_t265/stereo_imu.yaml 
roslaunch vins vins_rviz.launch
```

就可以开始愉快的跑VINS-Fusion了。  
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200225182050146.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80NDYzMTE1MA==,size_16,color_FFFFFF,t_70)



## 遇到的问题：

1. 老是漂移 参数差一点都不行

# 参考文章：

[Realsense T265标定及运行VINS–kalibr和imu_utils](https://www.jianshu.com/p/194d6c9ef9a4)  
[如何用Realsense D435i运行VINS-Mono等VIO算法 获取IMU同步数据](https://blog.csdn.net/qq_41839222/article/details/86552367)  
[使用imu_utils工具生成IMU的Allan方差标定曲线](https://blog.csdn.net/u011392872/article/details/95787486)  
[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)