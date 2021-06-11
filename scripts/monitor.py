#!/usr/bin/python3
#-*- coding: utf-8 -*-

import copy
import sys
import threading
import time
import struct
import math
import nnpy
import json
import numpy as np
import matplotlib.pyplot as plt
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg



f = open("../params/nano_msg.json")
content = f.read()
configs = json.loads(content)
print(configs)



NNG_URL_NAV_POSE_START = configs['NNG_URL_NAV_POSE_START_SUB']
NNG_URL_NAV_POSE_GOAL = configs['NNG_URL_NAV_POSE_GOAL_SUB']
NNG_URL_NAV_GLOBAL_PLAN = configs['NNG_URL_NAV_GLOBAL_PLAN_SUB']
#NNG_URL_NAV_GLOBAL_PLAN = configs['NNG_URL_NAV_LOCAL_PLAN_SUB']
NNG_URL_NAV_LOCAL_PLAN = configs['NNG_URL_NAV_LOCAL_PLAN_SUB']
NNG_URL_ODOM = configs['NNG_URL_NAV_ODOM_SUB']
NNG_URL_VELOCITY = configs['NNG_URL_ROBOT_TRACK_VELOCITY_SUB']
#NNG_URL_VELOCITY = configs['NNG_URL_ROBOT_ACTUAL_VELOCITY_SUB']
NNG_URL_TRACK_POSE = configs['NNG_URL_ROBOT_TRACK_POSE_SUB']

TRACK_POSE_X_LIST = np.array([])
TRACK_POSE_Y_LIST = np.array([])
VELOCITY_LIST = np.array([])
STEER_ANGLE_LIST = np.array([])
STEER_ACC_LIST = np.array([])
GLOBAL_PATH_X_LIST = np.array([])
GLOBAL_PATH_Y_LIST = np.array([])
GLOBAL_PATH_YAW_LIST = np.array([])
PATH_X_LIST = np.array([])
PATH_Y_LIST = np.array([])
PATH_YAW_LIST = np.array([])
ODOM_X_LIST = np.array([])
ODOM_Y_LIST = np.array([])
ODOM_YAW_LIST = np.array([])

MUTEX = threading.RLock()


def create_sub_socket(url):
    sub = nnpy.Socket(nnpy.AF_SP, nnpy.SUB)
    sub.connect(url)
    sub.setsockopt(nnpy.SUB, nnpy.SUB_SUBSCRIBE, '')
    return sub

class Pose:
    def __init__(self, ix, iy, iangle):
        self.x = ix
        self.y = iy
        self.angle = iangle

class OnGoalChanged(threading.Thread):
    def __init__(self, url):
        threading.Thread.__init__(self)
        self.sub = create_sub_socket(url)
    def run(self):
        global TRACK_POSE_X_LIST
        global TRACK_POSE_Y_LIST
        global MUTEX
        global VELOCITY_LIST
        global STEER_ANGLE_LIST
        global STEER_ACC_LIST
        global PATH_X_LIST
        global PATH_Y_LIST
        global PATH_YAW_LIST
        global ODOM_X_LIST
        global ODOM_Y_LIST
        global ODOM_YAW_LIST
        while(True):
            recv_data = self.sub.recv()
            print("recv new goal, clear data")
            MUTEX.acquire()
            TRACK_POSE_X_LIST = np.array([])
            TRACK_POSE_Y_LIST = np.array([])
            VELOCITY_LIST = np.array([])
            STEER_ANGLE_LIST = np.array([])
            STEER_ACC_LIST = np.array([])
            GLOBAL_PATH_X_LIST = np.array([])
            GLOBAL_PATH_Y_LIST = np.array([])
            GLOBAL_PATH_YAW_LIST = np.array([])
            PATH_X_LIST = np.array([])
            PATH_Y_LIST = np.array([])
            PATH_YAW_LIST = np.array([])
            ODOM_X_LIST = np.array([])
            ODOM_Y_LIST = np.array([])
            ODOM_YAW_LIST = np.array([])
            MUTEX.release()

class ControlThread (threading.Thread):
    def __init__(self, url):
        threading.Thread.__init__(self)
        self.sub = create_sub_socket(url)
    def run(self):
        global MUTEX
        global VELOCITY_LIST
        global STEER_ANGLE_LIST
        global STEER_ACC_LIST
        while(True):
            recv_data = self.sub.recv()
            #print(recv_data)
            velocity = struct.unpack('<d', recv_data[-17:-9])[0]
            steer_angle = struct.unpack('<d', recv_data[-9:-1])[0] * 180 / math.pi
            MUTEX.acquire()
            if velocity != 0 or steer_angle != 0:
                VELOCITY_LIST = np.append(VELOCITY_LIST, velocity)
                STEER_ANGLE_LIST = np.append(STEER_ANGLE_LIST, steer_angle)
            
            MUTEX.release()

class TrackPoseThread(threading.Thread):
    def __init__(self, url):
        threading.Thread.__init__(self)
        self.sub = create_sub_socket(url)
    def run(self):
        global MUTEX
        global TRACK_POSE_X_LIST
        global TRACK_POSE_Y_LIST
        while(True):
            recv_data = self.sub.recv()
            x = struct.unpack('<d', recv_data[-25:-17])[0]
            y = struct.unpack('<d', recv_data[-17:-9])[0]
            angle = struct.unpack('<d', recv_data[-9:-1])[0] * 180 / math.pi
            pose = Pose(x, y, angle)
            MUTEX.acquire()
            if x != 0  or y != 0:
                TRACK_POSE_X_LIST = np.append(TRACK_POSE_X_LIST, x)
                TRACK_POSE_Y_LIST = np.append(TRACK_POSE_Y_LIST, y)
            MUTEX.release()


class GlobalPathThread(threading.Thread):
    def __init__(self, url):
        threading.Thread.__init__(self)
        self.sub = create_sub_socket(url)
    def run(self):
        boost_header_size = 68 + 8 + 5
        global MUTEX
        global GLOBAL_PATH_X_LIST
        global GLOBAL_PATH_Y_LIST
        global GLOBAL_PATH_YAW_LIST
        while(True):
            recv_data = self.sub.recv()
            data = recv_data[boost_header_size-1:-1]
            path_size = len(data)
            #print("%d\n\n" %path_size)
            MUTEX.acquire()
            if path_size > 0:
                print("\nreceive global path")
                GLOBAL_PATH_X_LIST = np.array([])
                GLOBAL_PATH_Y_LIST = np.array([])
                GLOBAL_PATH_YAW_LIST = np.array([])
            for i in range(path_size//24):
                index = i * 24
                x = struct.unpack('<d', data[index:index+8])[0]
                y = struct.unpack('<d', data[index+8:index+16])[0]
                heading_angle = struct.unpack('<d', data[index+16:index+24])[0] * 180 / math.pi
                #print("%.3f, %.3f, %.3f" %(x, y, heading_angle * math.pi / 180.0))
                GLOBAL_PATH_X_LIST = np.append(GLOBAL_PATH_X_LIST, x)
                GLOBAL_PATH_Y_LIST = np.append(GLOBAL_PATH_Y_LIST, y)
                GLOBAL_PATH_YAW_LIST = np.append(GLOBAL_PATH_YAW_LIST, heading_angle)
                #print(len(PATH_X_LIST))
            MUTEX.release()

class PathThread(threading.Thread):
    def __init__(self, url):
        threading.Thread.__init__(self)
        self.sub = create_sub_socket(url)
    def run(self):
        boost_header_size = 68 + 8 + 5
        global MUTEX
        global PATH_X_LIST
        global PATH_Y_LIST
        global PATH_YAW_LIST
        while(True):
            recv_data = self.sub.recv()
            data = recv_data[boost_header_size-1:-1]
            path_size = len(data)
            
            MUTEX.acquire()
            for i in range(path_size//24):
                index = i * 24
                x = struct.unpack('<d', data[index:index+8])[0]
                y = struct.unpack('<d', data[index+8:index+16])[0]
                heading_angle = struct.unpack('<d', data[index+16:index+24])[0] * 180 / math.pi
                print("%.3f,\t%.3f,\t%.3f\t(%.3f)" %(x, y, heading_angle * math.pi / 180.0, heading_angle))
                PATH_X_LIST = np.append(PATH_X_LIST, x)
                PATH_Y_LIST = np.append(PATH_Y_LIST, y)
                PATH_YAW_LIST = np.append(PATH_YAW_LIST, heading_angle)
                #print(len(PATH_X_LIST))
            MUTEX.release()
            
class OdomThread (threading.Thread):
    def __init__(self, url):
        threading.Thread.__init__(self)
        self.sub = create_sub_socket(url)
    def run(self):
        global MUTEX
        global ODOM_X_LIST
        global ODOM_Y_LIST
        global ODOM_YAW_LIST
        while(True):
            recv_data = self.sub.recv()
            x = struct.unpack('<d', recv_data[-25:-17])[0]
            y = struct.unpack('<d', recv_data[-17:-9])[0]
            heading_angle = struct.unpack('<d', recv_data[-9:-1])[0] * 180 / math.pi
            MUTEX.acquire()
            ODOM_X_LIST = np.append(ODOM_X_LIST, x)
            ODOM_Y_LIST = np.append(ODOM_Y_LIST, y)
            ODOM_YAW_LIST = np.append(ODOM_YAW_LIST, heading_angle)
            MUTEX.release()



# main
thread_list = []
thread_list.append(ControlThread(NNG_URL_VELOCITY))
thread_list.append(GlobalPathThread(NNG_URL_NAV_GLOBAL_PLAN))
thread_list.append(PathThread(NNG_URL_NAV_LOCAL_PLAN))
thread_list.append(OdomThread(NNG_URL_ODOM))
thread_list.append(OnGoalChanged(NNG_URL_NAV_POSE_GOAL))
thread_list.append(TrackPoseThread(NNG_URL_TRACK_POSE))
for i in thread_list:
    i.start()


app = QtGui.QApplication([])

win_control = pg.GraphicsWindow(title="Control")
win_control.resize(800,600)
win_control.setWindowTitle('Control')
win_path = pg.GraphicsWindow(title="Path")
win_path.resize(800,600)
win_path.setWindowTitle('Path')

plot_control_velocity = win_control.addPlot(title="Velocity")
win_control.nextRow()
plot_control_steer_angle = win_control.addPlot(title="Steer Angle")
# win_control.nextRow()
# plot_control_steer_acc = win_control.addPlot(title="Steer Angle Acc")

curve_velocity = plot_control_velocity.plot()
curve_steer_angle = plot_control_steer_angle.plot()
#curve_steer_acc = plot_control_steer_acc.plot()

plot_path = win_path.addPlot(title="Path")

def update():
    global VELOCITY_LIST
    global STEER_ANGLE_LIST
    global STEER_ACC_LIST
    global GLOBAL_PATH_X_LIST
    global GLOBAL_PATH_Y_LIST
    global GLOBAL_PATH_YAW_LIST
    global PATH_X_LIST
    global PATH_Y_LIST
    global PATH_YAW_LIST
    global ODOM_X_LIST
    global ODOM_Y_LIST
    global ODOM_YAW_LIST
    MUTEX.acquire()
    # plot_control_steer_acc.clear()
    # plot_control_steer_angle.clear()
    # plot_control_velocity.clear()
    plot_path.clear()
    global curve_velocity
    curve_velocity.setData(VELOCITY_LIST)
    curve_steer_angle.setData(STEER_ANGLE_LIST)
    #curve_steer_acc.setData(STEER_ACC_LIST)
    # can be one (or a list) of: 
    # * ‘o’ circle (default) * ‘s’ square * ‘t’ triangle 
    # * ‘d’ diamond * ‘+’ plus * any QPainterPath to specify custom symbol shapes. 
    # To properly obey the position and size, 
    # custom symbols should be centered at (0,0) and 
    # width and height of 1.0. 
    # Note that it is also possible to ‘install’ 
    # custom shapes by setting ScatterPlotItem.Symbols[key] = shape.
    #plot_path.plot(GLOBAL_PATH_X_LIST, GLOBAL_PATH_Y_LIST, pen='g', symbol='o', symbolSize=10, symbolPen=(0,255,0,255), symbolBrush=(0,255,0,255))
    plot_path.plot(GLOBAL_PATH_X_LIST, GLOBAL_PATH_Y_LIST, pen='g')
    #plot_path.plot(PATH_X_LIST, PATH_Y_LIST, pen='r')
    plot_path.plot(TRACK_POSE_X_LIST, TRACK_POSE_Y_LIST, pen='r', symbol='o', symbolSize=10, symbolPen=(200,0,0,150), symbolBrush=(200,0,0,150))
    #plot_path.plot(PATH_X_LIST, PATH_Y_LIST, pen=None, symbol='o', symbolSize=10, symbolPen=(255,255,255,200), symbolBrush=(255,0,0,150))
    #plot_path.plot(ODOM_X_LIST, ODOM_Y_LIST, pen='g', symbol='o')
    plot_path.plot(ODOM_X_LIST, ODOM_Y_LIST, pen='b')
    MUTEX.release()

timer = QtCore.QTimer() 
timer.timeout.connect(update)
timer.start(300)

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
