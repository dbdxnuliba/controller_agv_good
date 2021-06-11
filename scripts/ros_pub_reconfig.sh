#!/bin/bash
source /opt/ros/melodic/setup.sh 
rostopic pub -1 /BZRobotReconfig std_msgs/String "./params/configs.json" 