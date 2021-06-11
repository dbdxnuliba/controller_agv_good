#!/usr/bin/python3
#-*- coding: utf-8 -*-

import subprocess
import signal
import sys
import time
import os


ld_path = os.getenv('LD_LIBRARY_PATH')
lib_bz_robot_path = os.getcwd() + '/build'
lib_osqp_eigen_path = os.getcwd() + '/third_libs/osqp-eigen-0.6.2/build'
is_new_ld_path = False
if ld_path is None:
    os.environ['LD_LIBRARY_PATH'] = lib_bz_robot_path
    os.environ['LD_LIBRARY_PATH'] += ':' + lib_osqp_eigen_path
    is_new_ld_path = TRUE
else:
    if not lib_bz_robot_path in os.environ['LD_LIBRARY_PATH']:
        os.environ['LD_LIBRARY_PATH'] += ':' + lib_bz_robot_path
        is_new_ld_path = True
    if not lib_osqp_eigen_path in os.environ['LD_LIBRARY_PATH']:
        os.environ['LD_LIBRARY_PATH'] += ':' + lib_osqp_eigen_path
        is_new_ld_path = True


if is_new_ld_path:
    try:
        os.execv(sys.argv[0], sys.argv)
    except Exception as exc:
        print('Failed re-exec:', exc)
        sys.exit(2)


os.chdir("./..")
os.system('ulimit -c unlimited')
sub_process_list = []

def close_all():
    for i in range(len(sub_process_list)):
        # 相当于 kill -9
        sub_process_list[i].kill()
        # 相当于 kill
        #sub_process_list[i].terminate()
try:
    sub_process_list.append(subprocess.Popen('roscore'))
    sub_process_list.append(subprocess.Popen(['rviz', '-d', './rviz_config.rviz']))
    time.sleep(2)
    '''
    #sub_process_list.append(subprocess.Popen(['./build/devel/lib/bz_robot/map_to_odom']))
    sub_process_list.append(subprocess.Popen(['./build/collision_detector_server']))
    sub_process_list.append(subprocess.Popen(['./build/localization_server']))
    sub_process_list.append(subprocess.Popen(['./build/map_server']))
    sub_process_list.append(subprocess.Popen(['./build/planner_server']))
    #sub_process_list.append(subprocess.Popen('valgrind --tool=memcheck ./build/service_server ./params/configs.json', shell = True,))
    sub_process_list.append(subprocess.Popen(['./build/tracker_server']))
    sub_process_list.append(subprocess.Popen(['./build/robot_server']))
    sub_process_list.append(subprocess.Popen(['./build/smoother_server']))
    sub_process_list.append(subprocess.Popen(['./build/service_server', './params/configs_simulate.json']))
    time.sleep(5)
    #sub_process_list.append(subprocess.Popen(['./build/task_manager', './params/configs.json']))
    # ros plugins
    sub_process_list.append(subprocess.Popen(['./build/map_to_odom']))
    sub_process_list.append(subprocess.Popen(['./build/ros_interface']))
    time.sleep(2)
    #sub_process_list.append(subprocess.Popen([stage]))
    '''
    sub_process_list.append(subprocess.Popen(['./build/map_to_odom']))
    bz_robot = subprocess.Popen(['./build/bz_robot', './params/configs_simulate.json'])
    sub_process_list.append(bz_robot)
    
except Exception as e:
    print("\n\n EXCEPTION OCCURED:")
    print(e)
    close_all()

def signal_handler(signal, frame):
    close_all()
    sys.exit(0)
 

signal.signal(signal.SIGINT,signal_handler)

print("wait bz robot finish")
bz_robot.wait()
print("\n\n\nbz robot exit error\n\n\n\n\n")
close_all()
print('close')


# while True:
#     time.sleep(10)
    #pass