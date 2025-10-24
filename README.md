# HAMHAM

## 폴더 및 파일 구조도
```mermaid
%%{init: {"flowchart": {
  "curve": "linear",
  "htmlLabels": true,
  "useMaxWidth": false,
  "nodeSpacing": 16,
  "rankSpacing": 16,
  "padding": 0
}} }%%
graph LR
classDef node fill:#eef,stroke:#99c,color:#111,stroke-width:1px;
classDef topic fill:#f4f1ff,stroke:#99c,color:#333,stroke-width:1px;

%% Localization
subgraph Localization
direction LR
  U[<b>ublox_gps_node</b><br/>ublox/ublox_gps/src/node.cpp]:::node
  Fix((/ublox_gps_node/fix)):::topic
  NMEA((/nmea)):::topic
  N[<b>ntrip_client</b><br/>ntrip_client/scripts/ntrip_ros.py]:::node
  RTCM((/rtcm)):::topic
  U --> Fix
  U --> NMEA
  Fix --> N
  NMEA --> N
  N --> RTCM
  RTCM --> U
end

%% IMU
subgraph IMU
direction LR
  B[<b>bno055</b><br/>combined_rtk/combined_rtk/bno055.py]:::node
  Yaw((/bno055/yaw_deg)):::topic
  Cal((/bno055/calibration)):::topic
  Ready((/bno055/is_calibrated)):::topic
  Debug((/bno055/serial_debug)):::topic
  B --> Yaw
  B --> Cal
  B --> Ready
  B -.-> Debug
end

%% State Estimation
subgraph State_Estimation
direction LR
  P[<b>pose_publisher</b><br/>gps_path_planner/gps_path_planner/pose_publisher.py]:::node
  XY((/current_xy)):::topic
  YT((/current_yaw)):::topic
  Fix -->|fix| P
  Yaw -->|yaw| P
  P --> XY
  P --> YT
end

%% Planning / Control
subgraph Planning
direction LR
  G[<b>gps_global_path_publisher</b><br/>gps_path_planner/gps_path_planner/gps_publish_global_path_node.py]:::node
  PATH((/global_path)):::topic
  G --> PATH

  PP[<b>pure_pursuit_node</b><br/>gps_path_planner/gps_path_planner/pure_pursuit_node.py]:::node
  MM[<b>mission_manager</b><br/>gps_path_planner/gps_path_planner/mission_manager.py]:::node

  %% XY / Yaw → 두 노드로 각각 라벨 표시
  XY -->|XY| PP
  XY -->|XY| MM
  YT -->|Yaw| PP
  YT -->|Yaw| MM

  Throttle((/drive_throttle)):::topic --> PP
  Throttle --> MM
  CMD((/cmd_vel)):::topic
  PP --> CMD
  MM --> CMD
end

subgraph Control_IO
direction LR
  TSB[<b>twist_serial_bridge</b><br/>gps_path_planner/gps_path_planner/twist_serial_bridge.py]:::node
  CMD --> TSB
  Throttle --> TSB
end

%% Lidar
subgraph Perception_Lidar
direction LR
  LIDAR[<b>rplidar_node</b><br/>rplidar_ros/src/rplidar_node.cpp]:::node
  SCAN((/scan)):::topic
  LIDAR --> SCAN
  LidarROI[<b>lidar_roi_node</b><br/>gps_path_planner/gps_path_planner/lidar_roi_node.py]:::node
  SCAN --> LidarROI
  FRONT((/scan/front_filtered)):::topic
  OBST((/obstacles/centers)):::topic
  ROI((/obstacles/roi_centers)):::topic
  MIND((/obstacles/roi_min_distance)):::topic
  ALERT((/obstacles/roi_alert)):::topic
  LidarROI --> FRONT
  LidarROI --> OBST
  LidarROI --> ROI
  LidarROI --> MIND
  LidarROI --> ALERT
end

%% Cameras
subgraph Perception_Cameras
direction LR
  LC[<b>left_cam/camera_publisher</b><br/>gps_path_planner/gps_path_planner/camera_publisher.py]:::node --> LeftImg((/left_cam/image_raw)):::topic
  CC[<b>center_cam/camera_publisher</b><br/>gps_path_planner/gps_path_planner/camera_publisher.py]:::node --> CenterImg((/center_cam/image_raw)):::topic

  ConeL[<b>left_cam/cone_detector</b><br/>gps_path_planner/gps_path_planner/cone_detector.py]:::node
  LeftImg --> ConeL
  CLSL((/cones/left/state)):::topic
  IML((/cones/left/image)):::topic
  ConeL --> CLSL
  ConeL --> IML

  ConeC[<b>center_cam/cone_detector</b><br/>gps_path_planner/gps_path_planner/cone_detector.py]:::node
  CenterImg --> ConeC
  CLSC((/cones/center/state)):::topic
  IMC((/cones/center/image)):::topic
  ConeC --> CLSC
  ConeC --> IMC

  TL[<b>center_cam/traffic_light_node</b><br/>gps_path_planner/gps_path_planner/traffic_light_node.py]:::node
  CenterImg --> TL
  TLR((/traffic_light/red)):::topic
  TLG((/traffic_light/green)):::topic
  TLO((/traffic_light/orange)):::topic
  TLL((/traffic_light/left)):::topic
  TLIMG((/traffic_light/image)):::topic
  TL --> TLR
  TL --> TLG
  TL --> TLO
  TL --> TLL
  TL --> TLIMG
end

```
