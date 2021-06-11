#!/bin/bash
cd ../ros_ws/ws_hdl
source devel/setup.sh
cd src/hdl_localization/launch
roslaunch hdl_localization.launch
