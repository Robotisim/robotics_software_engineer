
# Module # 6 : Sensor FUsion ( V1.0 )
- Package Created : *sensor_fusion*
    - 1D and 2D Kalman Filter
    - EKF and UKF
    - Particle Filter
---

## Developing
### Run it
```
ros2 run ros2_kitti_publishers kitti_publishers
```
```
cbs && ros2 launch  sensor_fusion kitti_imu.launch.py
```
```
rviz2
```
### Stuck at
- Kitti IMU + GPS data sensor fusion using Robot_localization package

### Important Points
- EKF Parameter importance ( follow this as well : https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/)
    - If you run simple ekf node it will not subscribe things until you mention in the params file what to subscribe
    - If you donot give proper name to confifguration file in yaml file of the node it will not read the parameters properly
    - Understanding the purpose of true false , with below setting Problems and confusion because
    ```
        odom0_config: [false, false, false,  # Position XYZ
                    false, false, true,  # Orientation XYZ
                    true, true, true,  # Linear velocity XYZ
                    false, false, false,   # Angular velocity XYZ
                    false, false, false]  # Linear acceleration XYZ
    odom0_queue_size: 10
    odom0_nodelay: true
    odom0_differential: false
    odom0_relative: false

    imu0: /imu
    imu0_config: [false, false, false,
                  true, true, false,
                  false, false, false,
                  true, true, true,
                  true, true, true]
    ```
    - This got solved with . Specifically setting two_D mode true
    ```
      odom0_config: [true, true, false,  # Position XYZ
                    false, false, true,  # Orientation XYZ
                    false, false, false,  # Linear velocity XYZ
                    false, false, false,   # Angular velocity XYZ
                    false, false, false]  # Linear acceleration XYZ
    odom0_queue_size: 10
    odom0_nodelay: true
    odom0_differential: false
    odom0_relative: false

    imu0: /imu
    imu0_config: [false, false, false,
                  true, true, true,
                  false, false, false,
                  true, true, true,
                  true, true, true]
    ```