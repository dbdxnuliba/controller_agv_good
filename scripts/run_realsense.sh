#!/bin/bash

source ../ros_ws/ws_realsense/devel/setup.bash

roslaunch realsense2_camera rs_multiple_devices_juion.launch &

roslaunch laserscan_kinect laserscan.launch
