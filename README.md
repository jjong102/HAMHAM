# HAMHAM

## 구조도
```mermaid
graph LR

%% ========== Localization (GPS + NTRIP) ==========
subgraph Localization
direction LR
  U[ublox_gps_node (ublox/ublox_gps/src/node.cpp + ublox_firmware6.cpp)]
  N[ntrip_client (ntrip_client/scripts/ntrip_ros.py)]

  Fix((/ublox_gps_node/fix))
  NMEA((/nmea))
  RTCM((/rtcm))

  U --> Fix
  U --> NMEA
  Fix -->|sub: ntrip_client/scripts/ntrip_ros_base.py| N
  NMEA -->|sub: ntrip_client/scripts/ntrip_ros_base.py| N
  N -->|pub: ntrip_client/scripts/ntrip_ros_base.py| RTCM
  RTCM -->|sub: ublox/ublox_gps/src/node.cpp| U
end

%% ========== IMU (BNO055) ==========
subgraph IMU
direction LR
  B[bno055 (combined_rtk/combined_rtk/bno055.py)]
  Yaw((/bno055/yaw_deg))
  Cal((/bno055/calibration))
  Ready((/bno055/is_calibrated))
  Debug((/bno055/serial_debug))
  B --> Yaw
  B --> Cal
  B --> Ready
  B -. opt .-> Debug
end

%% ========== State Estimation ==========
subgraph State Estimation
direction LR
  P[pose_publisher (gps_path_planner/gps_path_planner/pose_publisher.py)]
  XY((/current_xy))
  Y((/current_yaw))
  Fix -->|sub: pose_publisher.py| P
  Yaw -->|sub: pose_publisher.py| P
  P --> XY
  P --> Y
end

%% ========== Planner / Control ==========
subgraph Planning
direction LR
  G[gps_global_path_publisher (gps_path_planner/gps_path_planner/gps_publish_global_path_node.py)]
  PATH((/global_path))
  G --> PATH

  PP[pure_pursuit_node (gps_path_planner/gps_path_planner/pure_pursuit_node.py)]
  XY -->|sub: pure_pursuit_node.py| PP
  Y  -->|sub: pure_pursuit_node.py| PP
  PATH -->|sub: pure_pursuit_node.py| PP
  Throttle((/drive_throttle)) -->|sub: pure_pursuit_node.py| PP
  CMD((/cmd_vel))
  PP --> CMD

  MM[mission_manager (gps_path_planner/gps_path_planner/mission_manager.py)]
  PATH -->|sub: mission_manager.py| MM
  XY   -->|sub: mission_manager.py| MM
  Y    -->|sub: mission_manager.py| MM
  Throttle -->|sub: mission_manager.py| MM
  MM --> CMD
end

subgraph Control IO
direction LR
  TSB[twist_serial_bridge (gps_path_planner/gps_path_planner/twist_serial_bridge.py)]
  CMD -->|sub: twist_serial_bridge.py| TSB
  Throttle -->|sub: twist_serial_bridge.py| TSB
end

%% ========== Perception (Lidar / Camera) ==========
subgraph Perception - Lidar
direction LR
  LIDAR[rplidar_node (rplidar_ros/src/rplidar_node.cpp)]
  SCAN((/scan))
  LIDAR --> SCAN
  LidarROI[lidar_roi_node (gps_path_planner/gps_path_planner/lidar_roi_node.py)]
  SCAN -->|sub: lidar_roi_node.py| LidarROI
  FRONT((/scan/front_filtered))
  OBST((/obstacles/centers))
  ROI((/obstacles/roi_centers))
  MIND((/obstacles/roi_min_distance))
  ALERT((/obstacles/roi_alert))
  LidarROI --> FRONT
  LidarROI --> OBST
  LidarROI --> ROI
  LidarROI --> MIND
  LidarROI --> ALERT
end

subgraph Perception - Cameras
direction LR
  LC[left_cam/camera_publisher (gps_path_planner/gps_path_planner/camera_publisher.py)] --> LeftImg((/left_cam/image_raw))
  CC[center_cam/camera_publisher (gps_path_planner/gps_path_planner/camera_publisher.py)] --> CenterImg((/center_cam/image_raw))

  ConeL[left_cam/cone_detector (gps_path_planner/gps_path_planner/cone_detector.py)]
  LeftImg -->|sub: cone_detector.py| ConeL
  ConeL --> CLSL((/cones/left/state))
  ConeL --> IML((/cones/left/image))

  ConeC[center_cam/cone_detector (gps_path_planner/gps_path_planner/cone_detector.py))]
  CenterImg -->|sub: cone_detector.py| ConeC
  ConeC --> CLSC((/cones/center/state))
  ConeC --> IMC((/cones/center/image))

  TL[center_cam/traffic_light_node (gps_path_planner/gps_path_planner/traffic_light_node.py)]
  CenterImg -->|sub: traffic_light_node.py| TL
  TL --> TLR((/traffic_light/red))
  TL --> TLG((/traffic_light/green))
  TL --> TLO((/traffic_light/orange))
  TL --> TLL((/traffic_light/left))
  TL --> TLIMG((/traffic_light/image))
end


```
