# HAMHAM

## 폴더 및 파일 구조도
```mermaid
%%{init: {"flowchart": {"curve": "stepAfter", "htmlLabels": true}} }%%
graph LR

subgraph Localization
  U[<b>ublox_gps_node</b><br/>ublox/ublox_gps/src/node.cpp<br/>ublox/ublox_gps/src/ublox_firmware6.cpp]
  N[<b>ntrip_client</b><br/>ntrip_client/scripts/ntrip_ros.py]

  Fix((/ublox_gps_node/fix))
  NMEA((/nmea))
  RTCM((/rtcm))

  U --> Fix
  U --> NMEA
  Fix --> N
  NMEA --> N
  N --> RTCM
  RTCM --> U
end

subgraph IMU
  B[<b>bno055</b><br/>combined_rtk/combined_rtk/bno055.py]
  Yaw((/bno055/yaw_deg))
  Cal((/bno055/calibration))
  Ready((/bno055/is_calibrated))
  Debug((/bno055/serial_debug))
  B --> Yaw
  B --> Cal
  B --> Ready
  B -.-> Debug
end

subgraph State_Estimation
  P[<b>pose_publisher</b><br/>gps_path_planner/gps_path_planner/pose_publisher.py]
  XY((/current_xy))
  Y((/current_yaw))
  Fix --> P
  Yaw --> P
  P --> XY
  P --> Y
end

subgraph Planning
  G[<b>gps_global_path_publisher</b><br/>gps_path_planner/gps_path_planner/gps_publish_global_path_node.py]
  PATH((/global_path))
  G --> PATH

  PP[<b>pure_pursuit_node</b><br/>gps_path_planner/gps_path_planner/pure_pursuit_node.py]
  PATH --> PP
  XY --> PP
  Y --> PP
  Throttle((/drive_throttle)) --> PP
  CMD((/cmd_vel))
  PP --> CMD

  MM[<b>mission_manager</b><br/>gps_path_planner/gps_path_planner/mission_manager.py]
  PATH --> MM
  XY --> MM
  Y --> MM
  Throttle --> MM
  MM --> CMD
end

subgraph Control_IO
  TSB[<b>twist_serial_bridge</b><br/>gps_path_planner/gps_path_planner/twist_serial_bridge.py]
  CMD --> TSB
  Throttle --> TSB
end

subgraph Perception_Lidar
  LIDAR[<b>rplidar_node</b><br/>rplidar_ros/src/rplidar_node.cpp]
  SCAN((/scan))
  LIDAR --> SCAN
  LidarROI[<b>lidar_roi_node</b><br/>gps_path_planner/gps_path_planner/lidar_roi_node.py]
  SCAN --> LidarROI
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

subgraph Perception_Cameras
  LC[<b>left_cam/camera_publisher</b><br/>gps_path_planner/gps_path_planner/camera_publisher.py] --> LeftImg((/left_cam/image_raw))
  CC[<b>center_cam/camera_publisher</b><br/>gps_path_planner/gps_path_planner/camera_publisher.py] --> CenterImg((/center_cam/image_raw))

  ConeL[<b>left_cam/cone_detector</b><br/>gps_path_planner/gps_path_planner/cone_detector.py]
  LeftImg --> ConeL
  CLSL((/cones/left/state))
  IML((/cones/left/image))
  ConeL --> CLSL
  ConeL --> IML

  ConeC[<b>center_cam/cone_detector</b><br/>gps_path_planner/gps_path_planner/cone_detector.py]
  CenterImg --> ConeC
  CLSC((/cones/center/state))
  IMC((/cones/center/image))
  ConeC --> CLSC
  ConeC --> IMC

  TL[<b>center_cam/traffic_light_node</b><br/>gps_path_planner/gps_path_planner/traffic_light_node.py]
  CenterImg --> TL
  TLR((/traffic_light/red))
  TLG((/traffic_light/green))
  TLO((/traffic_light/orange))
  TLL((/traffic_light/left))
  TLIMG((/traffic_light/image))
  TL --> TLR
  TL --> TLG
  TL --> TLO
  TL --> TLL
  TL --> TLIMG
end


```
