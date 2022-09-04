```bash
rosrun kalibr  kalibr_calibrate_cameras --target /home/eric/slam_ws/src/VINS-Fusion/config/realsense_t265/apriltags.yaml --bag /home/eric/slam_ws/cameras_calibration.bag --models omni-radtan omni-radtan --topics /fisheye1 /fisheye2

cd /home/eric/slam_ws/src/VINS-Fusion/config/realsense_t265
rosrun kalibr  kalibr_calibrate_imu_camera --target ./apriltags.yaml --cam ./cameras_calibration-camchain.yaml --imu ./imu.yaml --bag ./imu_cameras_calibration.bag --max-iter 30 --show-extraction

```



```
roslaunch realsense2_camera rs_t265.launch
rosrun vins vins_node ~/slam_ws/src/VINS-Fusion/config/realsense_t265/stereo_imu.yaml 
roslaunch vins vins_rviz.launch

rosbag record /camera/imu /camera/fisheye1/image_raw /camera/fisheye2/image_raw



————————————————
版权声明：本文为CSDN博主「IATBOMSW」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/weixin_44631150/article/details/104495156
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