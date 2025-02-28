sudo apt update
rosdep install --from-path src/zed_ros2_minimal --ignore-src -y
colcon build --symlink-install --base-paths  src/zed_ros2_minimal --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
source install/setup.bash

ros2 launch zed_ros2_minimal zed_camera.launch.py camera_model:=zedm