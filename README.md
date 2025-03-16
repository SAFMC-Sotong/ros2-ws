# ros2-ws
Minimal ROS2 workspace to get started

1. Setup workspace
```
git clone https://github.com/SAFMC-Sotong/ros2-ws.git
cd ros2-ws
mkdir src
mv mavros_bridge src/mavros_bridge
mv uosm_robot_viewer src/uosm_robot_viewer
mv zed_ros2_minimal src/zed_ros2_minimal
mv ros_deep_learning src/ros_deep_learning
mv stream_data src/stream_data
mv udp_mavros_bridge src/udp_mavros_bridge
```

2. Get dependencies
```
sudo apt get install ros-humble-mavros
./src/mavros_bridge/install_geolib.sh
```

3. Build and source
```
sudo apt update
rosdep install --from-path src --ignore-src -y
colcon build --symlink-install --base-paths  src --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
source install/setup.bash
```

4. Launch
```
ros2 launch mavros_bridge mavros_bridge.launch.py
```
## Systemctl setup 
### csi_stream.service
1. Go to the directory
```
cd /etc/systemd/system
sudo gedit csi_stream.service
```
2. Paste the following
```
[Unit]
Description=Launch CSI streaming
After=network.target

[Service]
Type=simple
Environment="HOME=/home/nvidia" # HOME_DIR
Environment="USER=nvidia" #USER
ExecStart=/usr/bin/bash /home/nvidia/workspaces/ros2/csi.sh # where the actual sh script is
WorkingDirectory=/home/nvidia/workspaces/ros2-ws
Restart=on-failure
User=nvidia #USER
Group=nvidia #USER
# Set the ROS 2 workspace environment variables if necessary

[Install]
WantedBy=multi-user.target

```
4. Save and exit, then.
```
sudo systemctl daemon-reload
sudo systemctl enable csi_stream.service
# To check if the service is running 
sudo systemctl status csi_stream.service 


(Optional)
sudo systemctl start/stop/restart csi_stream.service

```
### Mavros_bridge.service
1. Go to the directory
```
cd /etc/systemd/system
sudo gedit mavros_bridge.service
```
2. Paste the following
```
[Unit]
Description=Launch MAVROS Bridge
After=network.target

[Service]
Type=simple
Environment="HOME=/home/nvidia" # HOME_DIR
Environment="USER=nvidia" #USER
ExecStart=/usr/bin/bash /home/nvidia/workspaces/ros2/1run.sh # where the actual sh script is
WorkingDirectory=/home/nvidia/workspaces/ros2-ws
Restart=on-failure
User=nvidia #USER
Group=nvidia #USER
# Set the ROS 2 workspace environment variables if necessary

[Install]
WantedBy=multi-user.target

```
4. Save and exit, then.
```
sudo systemctl daemon-reload
sudo systemctl enable mavros_bridge.service
# To check if the service is running 
sudo systemctl status mavros_bridge.service 


(Optional)
sudo systemctl start/stop/restart mavros_bridge.service

```
