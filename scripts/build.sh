#!/bin/bash
source /opt/ros/melodic/setup.sh 
cd ../build && cmake .. && make -j4 && cd ../scripts
