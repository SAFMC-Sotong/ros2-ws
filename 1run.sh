#!/usr/bin/bash

ros2 launch mavros_bridge mavros_bridge.launch.py &> mavros_bridge_out.log &

gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=320, height=240, format=NV12, framerate=30/1' ! nvvidconv flip-method=2 ! 'video/x-raw, format=I420' ! jpegenc quality=60 ! udpsink host=10.42.0.180 port=5000 sync=false &> video_viewer_out.log &

#ros2 launch stream_data stream_data.launch.py &> stream_out.log &


