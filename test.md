```bash
rosrun kalibr kalibr_calibrate_cameras --target /home/eric/slam_ws/src/VINS-Fusion/config/realsense_t265/apriltags.yaml --bag /home/eric/slam_ws/cameras_calibration.bag --models omni-radtan omni-radtan --topics /fisheye1 /fisheye2

cd /home/eric/slam_ws/src/VINS-Fusion/config/realsense_t265
rosrun kalibr  kalibr_calibrate_imu_camera --target ./apriltags.yaml --cam ./cameras_calibration-camchain.yaml --imu ./imu.yaml --bag ./imu_cameras_calibration.bag --max-iter 30 --show-extraction

```



```
roslaunch realsense2_camera rs_t265.launch
rosrun vins vins_node ~/slam_ws/src/VINS-Fusion/config/realsense_t265/stereo_imu.yaml 
roslaunch vins vins_rviz.launch

rosbag record /camera/imu /camera/fisheye1/image_raw /camera/fisheye2/image_raw

```

https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/issues/57

https://www.jianshu.com/p/194d6c9ef9a4

https://zhuanlan.zhihu.com/p/480233374

https://zhuanlan.zhihu.com/p/414047132
fx fy其中为相机在的焦距, u0 v0为主点的坐标(像素坐标系原点相对于归一化平面原点的偏移).


https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/issues/48
body_T_cam0 means when you are sitting on body (imu)， the pose (position and orientation) of camera0 (left camera).
So, body_T_cam0 is the pose of the cam0 as observed from body frame aka imu.

p = body_T_cam0 * q

p: point in IMU coordinates
q: point in camera coordinates


## 自定义串口

https://blog.csdn.net/qq_39607707/article/details/125061020

### 2 运行步骤

1. 在运行程序之前，需要先在ubuntu系统下安装串口驱动。安装驱动的方法如下：新打开一个命  
    令窗口，输入以下命令即可。

```cpp
sudo apt install ros-noetic-serial
```

> 以上命令中的noetic是所装ROS的版本，如果所安装的ROS版本不是noetic，则需要把命令中的noetic改为自己装的ROS版本

2. 连接硬件设备
3. 查找对应串口的标号，在命令窗口输入以下命令即可查找对应串口的标号。

```cpp
ls -l /dev/ttyUSB* //查找串口标号
```

## 采集图像
```bash
rosbag record -O cameras_calibration_mono /usb_cam/image_raw

rosrun kalibr kalibr_calibrate_cameras --target ./apriltags.yaml --bag ~/cameras_calibration_mono.bag --models omni-radtan --topics /usb_cam/image_raw


# rosrun topic_tools throttle messages /camera/fisheye1/image_raw 20.0 /fisheye1
rosrun topic_tools throttle messages /imu_data 100.0 /imu

rosrun kalibr kalibr_calibrate_imu_camera --target ./apriltags.yaml --cam ./cameras_calibration_mono-camchain.yaml --imu ./imu.yaml --bag ~/imu_cameras_calibration_mono.bag --max-iter 30 --show-extraction

```