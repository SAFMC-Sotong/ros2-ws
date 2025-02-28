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
