robot_localization
==================

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://wiki.ros.org/robot_localization

## 동작 명령어

```
ros2 launch robot_localization scv_dual_ekf_navsat_example.launch.py
```

## 토픽

아래에 명시된 토픽만 우선 맞춰주면 됨

### input
- navsat_transform Node
  - /imu/data
    - IMU 센서 데이터
  - /gps/fix
    - GPS 센서 데이터
- ekf_filter_node_map Node
  - /imu/data
    - IMU 센서 데이터
  - /ackermann_like_controller/odom
    - 차량 Odometry 데이터
- ekf_filter_node_odom Node
  - /imu/data
    - IMU 센서 데이터
  - /ackermann_like_controller/odom
    - 차량 Odometry 데이터

### output

위 명령어 실행으로 map -> odom Transform이 생성되며,
/odometry/local
/odometry/global
위 두 토픽은 각각 local, global 환경을 기준으로 한 보정된 Odometry를 의미함
