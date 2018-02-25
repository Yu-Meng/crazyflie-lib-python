# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the crazyflie at `URI` and runs a figure 8
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander


from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import atexit

import fcntl
import sys
import os
# import tty
# import termios

import csv

import numpy as np
import cv2
import sys

import threading
from threading import Thread

SIDE_LENGTH = 16
PIXEL_LENGTH = 8
MARGIN = 20
GAP = 5
RADIUS = 5
TEXT_HEIGHT = 20
TEXT_HEIGHT_GAP = 2

VIDEO_Y_PIXEL = 480
VIDEO_X_PIXEL = 640

# B, G, R
COLOR = {'WHITE':(255, 255, 255), 'RED':(0, 0, 255), 'ORANGE':(0, 128, 255), 'YELLOW':(0, 255, 255), 'GREEN':(0, 255, 0), 'BLUE':(255, 0, 0), 'PINK':(255, 0, 255)}

redGroup = []
orangeGroup = []
yellowGroup = []
greenGroup = []

URI = 'usb://0'

TAKE_OFF = 0
CALIBRATE = 1
HOVER    = 2
LANDING  = 3
INTERRUPT = 4
FOLLOW   = 5
PREDICT_ROUTE = 6
MODE = ['TAKE OFF', 'CALIBRATE', 'HOVER', 'LANDING', 'INTERRUPT', 'FOLLOW', 'PREDICT_ROUTE']

DESIRED_HEIGHT_CM = 100
TIME_FACTOR   = 3

YAWRATE = 30 # degree/sec

mode = TAKE_OFF
last_mode = TAKE_OFF
trajectory = []
count = 0
key = ''
has_interrupt = False
has_emergency = False
has_done      = False

zrange = 0
range_front = 0
xw = 4.0
yw = 4.0
xh = 0.0
yh = 0.0
zMax = 0
zMin = 0
zMaxCollection = []
z_threshold_hover_high = 6
z_threshold_hover_low  = 4
z_threshold_follow_high = 5
z_threshold_follow_low  = 3
vx = 0.0
yawrate = 0.0


# object center
objectCX = 0
objectCY = 0

# Tracking mode
thermal_tracking_mode = False
face_tracking_mode = True


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

take_off_sequence = [
    # (0   , 0   , 0,  1/25),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0)
]

hover_sequence = [
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0)
]

landing_sequence = [
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0)
]

class raw(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()
    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)

class nonblocking(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()
    def __enter__(self):
        self.orig_fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl | os.O_NONBLOCK)
    def __exit__(self, *args):
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl)

def drawPixel(pixelNum, frame, color):
    row = int(pixelNum/8)
    col = pixelNum%8
    rotatedpixelNum = (PIXEL_LENGTH-1-col)*PIXEL_LENGTH + row
    rotatedRow = int(rotatedpixelNum/8)
    rotatedCol = rotatedpixelNum%8

    x_coordinate = VIDEO_X_PIXEL - (PIXEL_LENGTH - rotatedCol)*SIDE_LENGTH - (PIXEL_LENGTH - rotatedCol - 1)*GAP - MARGIN
    y_coordinate = VIDEO_Y_PIXEL - (PIXEL_LENGTH - rotatedRow)*SIDE_LENGTH - (PIXEL_LENGTH - rotatedRow - 1)*GAP - MARGIN
    cv2.rectangle(frame,(int(x_coordinate), int(y_coordinate)),(int(x_coordinate+SIDE_LENGTH), int(y_coordinate+SIDE_LENGTH)), color,-1)

def drawThermal(frame):
    for i in range(64):
        if i in redGroup:
            drawPixel(i, frame, COLOR['RED'])
        elif i in orangeGroup:
            drawPixel(i, frame, COLOR['ORANGE'])
        elif i in yellowGroup:
            drawPixel(i, frame, COLOR['YELLOW'])
        elif i in greenGroup:
            drawPixel(i, frame, COLOR['GREEN'])
        else:
            drawPixel(i, frame, COLOR['WHITE'])

def drawHeatCenter(frame, xw, yw, color):
    rotatedXw = PIXEL_LENGTH-1-xw
    rotatedYw = PIXEL_LENGTH-1-yw
    x_coordinate = VIDEO_X_PIXEL - (PIXEL_LENGTH - rotatedXw)*SIDE_LENGTH - (PIXEL_LENGTH - rotatedXw - 1)*GAP - MARGIN + SIDE_LENGTH/2
    y_coordinate = VIDEO_Y_PIXEL - (PIXEL_LENGTH - rotatedYw)*SIDE_LENGTH - (PIXEL_LENGTH - rotatedYw - 1)*GAP - MARGIN + SIDE_LENGTH/2
    # cv2.circle(frame, (int(50), int(150)), RADIUS, color, -1)
    cv2.circle(frame, (int(x_coordinate), int(y_coordinate)), RADIUS, color, -1)

def draw_text(frame):
    # x_coordinate = VIDEO_X_PIXEL - (PIXEL_LENGTH - 0)*SIDE_LENGTH - (PIXEL_LENGTH - 0 - 1)*GAP - MARGIN
    x_coordinate = 10
    # y_coordinate = VIDEO_Y_PIXEL - (PIXEL_LENGTH - 0)*SIDE_LENGTH - (PIXEL_LENGTH - 0 - 1)*GAP - MARGIN
    y_coordinate = VIDEO_Y_PIXEL - MARGIN

    font = cv2.FONT_HERSHEY_SIMPLEX
    fontsize = 0.6
    linesize = 1
    cv2.putText(frame,'zMax:'+str(zMax),(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-3*TEXT_HEIGHT), font, fontsize,(255,255,255),linesize,cv2.LINE_AA)
    cv2.putText(frame,'zMin:'+str(zMin),(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-2*TEXT_HEIGHT), font, fontsize,(255,255,255),linesize,cv2.LINE_AA)
    # cv2.putText(frame,'range_front_l:'+str(range_front),(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-4*TEXT_HEIGHT), font, fontsize,(255,255,255),linesize,cv2.LINE_AA)
    if thermal_tracking_mode:
        msg = 'Tag_x:{} Tag_y:{}'.format(xw, yw)
        cv2.putText(frame, msg,
                    (x_coordinate, y_coordinate - TEXT_HEIGHT_GAP -  TEXT_HEIGHT), font, fontsize, (255, 255, 255),
                    linesize, cv2.LINE_AA)
    if face_tracking_mode:
        msg = 'Obj_x:{} Obj_y:{}'.format(objectCX, objectCY)
        cv2.putText(frame, msg,
                    (x_coordinate, y_coordinate - TEXT_HEIGHT_GAP -  TEXT_HEIGHT), font, fontsize, (255, 255, 255),
                    linesize, cv2.LINE_AA)


    # cv2.putText(frame,'mode:'+MODE[mode],(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-3*TEXT_HEIGHT), font, fontsize,(255,255,255),linesize,cv2.LINE_AA)
    # msg = 'Z_THRES HOV:{}/{}'.format(z_threshold_hover_high, z_threshold_hover_low)
    # cv2.putText(frame,msg,(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-2*TEXT_HEIGHT), font, fontsize,(255,255,255),linesize,cv2.LINE_AA)
    # msg = '          FOL:{}/{}'.format(z_threshold_follow_high, z_threshold_follow_low)
    # cv2.putText(frame,msg,(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-TEXT_HEIGHT), font, fontsize,(255,255,255),linesize,cv2.LINE_AA)
    # msg = 'vx:{} yawrate:{}'.format(vx, yawrate)
    # cv2.putText(frame,msg,(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP), font, fontsize,(255,255,255),linesize,cv2.LINE_AA)
    timeArray = time.localtime()
    otherStyleTime = time.strftime("%Y-%m-%d %H:%M:%S", timeArray)
    cv2.putText(frame, otherStyleTime, (x_coordinate, y_coordinate - TEXT_HEIGHT_GAP), font, fontsize, (255, 255, 255), linesize,
                cv2.LINE_AA)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold_x = 0.001
    threshold_y = 0.001
    threshold_z = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold_x and (
                    max_y - min_y) < threshold_y and (
                    max_z - min_z) < threshold_z:
                break

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def change_mode(next_mode):
    global mode, count
    mode = next_mode
    count = 0
    trajectory_generator()

def check_interrupt():

    global mode, key, has_interrupt, has_emergency

    key = sys.stdin.read(1)
    if (key == 'l'):
        print("landing!!")
        change_mode(LANDING)
    elif (key == 'j'):
        has_interrupt = True
        print('left a bit')
    elif (key == 'k'):
        has_interrupt = True
        print('right a bit')
    elif (key == 'i'):
        has_interrupt = True
        print('forward a bit')
    elif (key == 'm'):
        has_interrupt = True
        print('backward a bit')
    elif (key == 'r'):
        has_interrupt = True
        print('rotate a bit')
    elif (key == 'e'):
        has_interrupt = True
        print('rotate back a bit')
    elif (key == 'q'):
        has_emergency = True
        print('emergency!!')

def handle_interrupt(action):
    global has_interrupt, key

    if mode == TAKE_OFF:
        if key == 'j':
            action = ( (action[0]-20), action[1], action[2], action[3] )
        elif key == 'k':
            action = ( (action[0]+20), action[1], action[2], action[3] )
        elif key == 'i':
            action = ( action[0], (action[1]-20), action[2], action[3] )
        elif key == 'm':
            action = ( action[0], (action[1]+20), action[2], action[3] )
        has_interrupt = False
        key = ''

    elif mode == CALIBRATE or mode == HOVER or mode == LANDING:
        if key == 'j':
            action = ( action[0], (action[1]+0.2), action[2], action[3] )
        elif key == 'k':
            action = ( action[0], (action[1]-0.2), action[2], action[3] )
        elif key == 'i':
            action = ( (action[0]+0.2), action[1], action[2], action[3] )
        elif key == 'm':
            action = ( (action[0]-0.2), action[1], action[2], action[3] )
        elif key == 'r':
            action = ( action[0], action[1], (action[2]+10), action[3] )
        elif key == 'e':
            action = ( action[0], action[1], (action[2]-10), action[3] )
        has_interrupt = False
        key = ''

    return action

def trajectory_generator():

    global trajectory, mode, vx, yawrate

    trajectory = []

    # 40 cm -> 10/25, 0.4 -> Desired_Height_CM=40, 10=Desired_Height_CM/4, 0.4=Desired_Height_CM/100
    # 80 cm -> 20/25, 0.8 -> Desired_Height_CM=80, 20=Desired_Height_CM/4, 0.8=Desired_Height_CM/100
    # 120 cm -> 30/25, 1.2 -> Desired_Height_CM=120, 30=Desired_Height_CM/4, 1.2=Desired_Height_CM/100

    if mode == TAKE_OFF:
        # for y in range(20):
        #     trajectory.append( (0,0,0, (y/50)) )

        for y in range( (int)((DESIRED_HEIGHT_CM/4.0)*TIME_FACTOR) ):
            trajectory.append( (0,0,0, (y/ (25*TIME_FACTOR))) )

    elif mode == CALIBRATE:
        # for _ in range(60):
        #     trajectory.append( (0,0,20,0.4) )

        for y in range(50):
            trajectory.append( (0,0,0,DESIRED_HEIGHT_CM/100) )

        # for _ in range(60):
        #     trajectory.append( (0,0,-20,0.8) )

        # for y in range(10):
        #     trajectory.append( (0,0,0, 0.8+(y/50)) )

    elif mode == HOVER:
        trajectory.append( (0,0,YAWRATE,DESIRED_HEIGHT_CM/100) )

    elif mode == FOLLOW:
        if abs(3.5-xw) > 2:
            vx = 0
            yawrate = YAWRATE

        else:
            # section                         yawrate
            # 0 <= abs(3.5-xw) < 0.7      ->  10
            # 0.7 <= abs(3.5-xw) < 1.4    ->  20
            # 1.4 <= abs(3.5-xw) < 2.1    ->  30
            # 2.1 <= abs(3.5-xw) < 2.8    ->  40
            # 2.8 <= abs(3.5-xw) < 3.5    ->  50

            yawrate_grid = 0.5
            yawrate_unit = 7.5 # degree/sec
            yawrate = round( int(abs(3.5-xw)/yawrate_grid+1) * yawrate_unit, 2)
            if (3.5-xw) < 0:
                yawrate = -yawrate

            detect_limit = 1200.0
            detect_grid = detect_limit / 10.0
            follow_distance = 500.0
            vx_unit = 0.1
            if range_front < detect_limit:
                vx = round( int(abs(range_front-follow_distance)/detect_grid+1) * vx_unit, 2)
                if range_front - follow_distance < 0:
                    vx = -vx
            else:
                vx = 0

        trajectory.append( (vx,0,yawrate,DESIRED_HEIGHT_CM/100) )

    elif mode == LANDING:
        for y in range( (int)((DESIRED_HEIGHT_CM/4.0)*TIME_FACTOR) ) :
            trajectory.append( (0,0,0, ( ((DESIRED_HEIGHT_CM/4)*TIME_FACTOR)-y ) / (25*TIME_FACTOR) ) )

    return trajectory

def action_update():

    global mode, trajectory, count, has_done

    if mode == TAKE_OFF:
        action = trajectory[count]
        count += 1
        return action

    elif mode == CALIBRATE:
        action = trajectory[count]
        count += 1
        return action

    elif mode == HOVER:
        return trajectory[count]

    elif mode == FOLLOW:
        trajectory = trajectory_generator()
        count = 0
        return trajectory[count]

    elif mode == LANDING:
        if count == (len(trajectory)-1):
            if zrange < 80:
                has_done = True
            return trajectory[count]
        action = trajectory[count]
        count += 1
        return action

def mode_update(cf):
    global mode, zMaxCollection, z_threshold_hover_high, z_threshold_hover_low, z_threshold_follow_high, z_threshold_follow_low

    if mode == TAKE_OFF:
        cf.param.set_value('zscore.thre_high', str(z_threshold_hover_high))
        cf.param.set_value('zscore.thre_low', str(z_threshold_hover_low))
        if count == len(trajectory):
            change_mode(CALIBRATE)

    elif mode == CALIBRATE:
        if count == len(trajectory):
            # avg = np.average(zMaxCollection)
            # std = np.std(zMaxCollection)
            peak = np.max(zMaxCollection)
            if z_threshold_hover_high < peak:
                diff = peak - z_threshold_hover_high
                z_threshold_hover_high += round(0.2*diff, 2)
                # z_threshold_hover_low += round(0.1*diff, 2)
                # z_threshold_follow_high += round(0.2*diff, 2)
                # z_threshold_follow_low += round(0.25*diff, 2)
            change_mode(HOVER)

        zMaxCollection.append(zMax)

    elif mode == HOVER:
        cf.param.set_value('zscore.thre_high', str(z_threshold_hover_high))
        cf.param.set_value('zscore.thre_low', str(z_threshold_hover_low))

        if abs(3.5-xw) < 2 and range_front < 1000:
            change_mode(FOLLOW)

    elif mode == FOLLOW:
        cf.param.set_value('zscore.thre_high', str(z_threshold_follow_high))
        cf.param.set_value('zscore.thre_low', str(z_threshold_follow_low))

        # if xw == -1:
        #     mode = PREDICT_ROUTE


def do_action(cf, action):

    global zrange, xw, range_front

    if has_interrupt:
        action = handle_interrupt(action)

    if has_emergency or has_done:
        f.close()
        sys.exit()

    if mode == TAKE_OFF:
        cf.commander.send_zdistance_setpoint(action[0], action[1], action[2], action[3])
        time.sleep(0.1)

    elif mode == CALIBRATE or mode == HOVER or mode == FOLLOW or mode == LANDING:
        cf.commander.send_hover_setpoint(action[0], action[1], action[2], action[3])
        time.sleep(0.1)

    print ("mode: ", MODE[mode])
    print ("next action: ", action[0], action[1], action[2], action[3])


def getBitfield(nL, nH):
    n = nL | (nH << 32)
    binStr = bin(n)[2:]    # remove '0b' of binary string
    rBitStr = binStr[::-1] # reverse binary string order
    rList = []
    for bit, digit in enumerate(rBitStr):
        if digit == '1':
            rList.append(bit)
    return rList

def setYellowGroup(yellowGroupL, yellowGroupH):
    global yellowGroup
    yellowGroup = getBitfield(yellowGroupL, yellowGroupH)
    # print('yellowGroup', yellowGroup)

def setOrangeGroup(orangeGroupL, orangeGroupH):
    global orangeGroup
    orangeGroup = getBitfield(orangeGroupL, orangeGroupH)
    # print('orangeGroup', orangeGroup)

def setRedGroup(redGroupL, redGroupH):
    global redGroup
    redGroup = getBitfield(redGroupL, redGroupH)
    # print('redGroup', redGroup)

def position_callback(timestamp, data, logconf):
    global zrange, range_front
    zrange = data['range.zrange']
    range_front = data['range.range_front_l']
    # print('range_front:{}, zrange:{}'.format(range_front, zrange))

def gridEyeXYZ_callback(timestamp, data, logconf):
    global xw, yw, xh, yh, zMax, zMin
    xw = data['gridEyeXYZ.xw']
    yw = data['gridEyeXYZ.yw']
    xh = data['gridEyeXYZ.xh']
    yh = data['gridEyeXYZ.yh']
    zMax = data['gridEyeXYZ.zMax'] / 100
    zMin = data['gridEyeXYZ.zMin'] / 100
    # print('xw:{}, yw:{}, xh:{}, yh:{}, zMax:{}, zMin:{}'.format(xw, yw, xh, yh, zMax, zMin))

def gridEye_callback(timestamp, data, logconf):
    setYellowGroup(data["gridEye.yellowGroupL"], data["gridEye.yellowGroupH"])
    setOrangeGroup(data["gridEye.orangeGroupL"], data["gridEye.orangeGroupH"])
    setRedGroup(data["gridEye.redGroupL"], data["gridEye.redGroupH"])

def start_logging(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('range.zrange', 'uint16_t')
    log_conf.add_variable('range.range_front_l', 'uint16_t')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

    log_conf = LogConfig(name='gridEyeXYZ', period_in_ms=100)
    log_conf.add_variable('gridEyeXYZ.xw', 'float')
    log_conf.add_variable('gridEyeXYZ.yw', 'float')
    log_conf.add_variable('gridEyeXYZ.xh', 'float')
    log_conf.add_variable('gridEyeXYZ.yh', 'float')
    log_conf.add_variable('gridEyeXYZ.zMax', 'int16_t')
    log_conf.add_variable('gridEyeXYZ.zMin', 'int16_t')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(gridEyeXYZ_callback)
    log_conf.start()

    log_conf = LogConfig(name='gridEye', period_in_ms=100)
    log_conf.add_variable("gridEye.yellowGroupH", "uint32_t")
    log_conf.add_variable("gridEye.yellowGroupL", "uint32_t")
    log_conf.add_variable("gridEye.orangeGroupH", "uint32_t")
    log_conf.add_variable("gridEye.orangeGroupL", "uint32_t")
    log_conf.add_variable("gridEye.redGroupH", "uint32_t")
    log_conf.add_variable("gridEye.redGroupL", "uint32_t")

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(gridEye_callback)
    log_conf.start()

def drawCV(vs):
    def draw():
        classifier = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
        while(1):
            # ret, frame = cap.read()
            time0 = time.time()
            ret, frame = vs.read()
            time1 = time.time()
            # print('ret:{}',ret)
            if ret==True:
                time2 = time.time()
                # frame = cv2.flip(frame,180)
                # write the flipped frame
                drawThermal(frame)
                drawHeatCenter(frame, xw, yw,  COLOR['GREEN'])
                drawHeatCenter(frame, xh, yh,  COLOR['PINK'])
                draw_text(frame)
                ################################face0################
                # size = frame.shape[:2]
                # image = np.zeros(size, dtype=np.float16)
                # image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # cv2.equalizeHist(image, image)
                # divisor = 8
                # h, w = size
                # # minSize=(w/divisor,h/divisor)
                # minSize = (int(w / divisor), int(h / divisor))
                # faceRects = classifier.detectMultiScale(image, 1.2, 2, cv2.CASCADE_SCALE_IMAGE, minSize)
                # if len(faceRects) > 0:
                #     for faceRect in faceRects:
                #         x, y, w, h = faceRect
                #         # x = int(x)
                #         # y = int(y)
                #         # w = int(w)
                #         # h = int(h)
                #         cv2.circle(frame, (int(x + w / 2), int(y + h / 2)), int(min(w / 2, h / 2)), (255, 0, 0))
            ######################################################
                time3 = time.time()
                ################################face1################
                vs.grabbed = False
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                time3_1 = time.time()
                faces = classifier.detectMultiScale(gray, 1.2, 5, 1)
                time3_2 = time.time()
                global objectCX
                objectCX = -1
                # objectCY = -1
                if len(faces) > 0:
                    for (x, y, w, h) in faces:
                        # frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        # roi_gray = gray[y:y + h, x:x + w]
                        # roi_color = frame[y:y + h, x:x + w]
                        objectCX = x + w / 2
                        objectCY = y + h / 2
                ######################################################
                time4 = time.time()




                # out.write(frame)
                # cv2.imshow('frame',frame)

                time5 = time.time()

                print('imgReceiving:{}'.format(time1 - time0))
                print('imgInitialization:{}'.format(time3 - time2))
                print('imgGray:{}'.format(time3_1 - time3))
                print('imgDetection:{}'.format(time3_2 - time3_1))
                print('imgDrawBox:{}'.format(time4 - time3_2))
                print('imgShow:{}'.format(time5 - time4))
                print('objectCX:{}'.format(objectCX))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break


        # cap.release()
        # vs.stop()
        # out.release()
        cv2.destroyAllWindows()

    opencv_thread = threading.Thread(target=draw, daemon=True)
    return opencv_thread

def automatic(scf, f):

    cf = scf.cf
    trajectory = trajectory_generator()

    while True:
        check_interrupt()
        action = action_update()
        do_action(cf, action)
        mode_update(cf)
        # write(f)

def init_hover():
    mc.up(0.6, velocity=0.2)
    time.sleep(3)

# def sensing():

# def dicision_maker():
#     if(xw >= 0 || xw < 2)



def selfcontrol():
    action_status = 0
    timing_status = 0
    timeOut = 5
    nothingStart = 0
    nothingEnd = 0
    print("start fly")
    while True:
        # sensing()
        # dicision_maker()
        # action_excutor()
        if thermal_tracking_mode :
            if action_status == 0:
                if xw >= 0 and xw < 2 :
                    if action_status == 0:
                        mc.start_turn_right(rate=30)
                        action_status = 1
                if xw >5 and xw <= 7 :
                        mc.start_turn_left(rate=30)
                        action_status = 2
            elif (xw >=2 and xw <= 5) or (xw < 0) :
                mc.stop()
                action_status = 0
            if xw < 0:
                if timing_status == 0:
                    nothingStart = time.time()
                    timing_status =1
                else:
                    nothingEnd = time.time()
            else:
                timing_status = 0

        if face_tracking_mode:

            if action_status == 0:
                if objectCX > 0 and objectCX < VIDEO_X_PIXEL * 0.35:
                    if action_status == 0:
                        mc.start_turn_left(rate=30)
                        action_status = 1
                if objectCX > VIDEO_X_PIXEL * 0.65 and objectCX <= VIDEO_X_PIXEL:
                    mc.start_turn_right(rate=30)
                    action_status = 2
            elif (objectCX >= VIDEO_X_PIXEL * 0.35 and objectCX <= VIDEO_X_PIXEL * 0.65) or (objectCX < 0):
                mc.stop()
                action_status = 0
            if objectCX < 0:
                if timing_status == 0:
                    nothingStart = time.time()
                    timing_status = 1
                else:
                    nothingEnd = time.time()
            else:
                timing_status = 0
        if nothingEnd - nothingStart > timeOut:
            break
        time.sleep(0.1)


###streaming thread

class VideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(3, VIDEO_X_PIXEL)
        self.stream.set(4, VIDEO_Y_PIXEL)
        self.stopped = False
        self.fps_array = []
        self.grabbed, self.frame = self.stream.read()
        # self.Q = Queue(maxsize=128)

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # otherwise, read the next frame from the stream

            # if not self.Q.full():
            #     start = cv2.getTickCount()
                # (self.grabbed, self.frame) = self.stream.read()
                # read the next frame from the file
            (self.grabbed, self.frame) = self.stream.read()
                # self.fps_array.append(cv2.getTickFrequency()/(cv2.getTickCount()-start))
                # if the `grabbed` boolean is `False`, then we have
                # reached the end of the video file
            if not self.grabbed:
                self.stop()
                return
                # add the frame to the Queue
                # self.Q.put(frame)


    def read(self):
        # return the frame most recently read
        return self.grabbed, self.frame

        # return next frame in the queue
        # return self.Q.get()

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def capture_fps(self):
        return sum(self.fps_array)/len(self.fps_array)





def write(f):
    global vPX, vPY, vPZ, zrange, sX, sY, sZ, zg, key
    # data_dict = {'key':key, 'vPX':vPX, 'vPY':vPY, 'vPZ':vPZ, 'zrange':zrange, 'sX':sX, 'sY':sY, 'sZ':sZ}
    # for key, value in data_dict.items():
    #     line
    #     line.append(value)
    msg = 'key:{}, sX:{}, sY:{}, sZ:{}, zrange:{}, vPX:{}, vPY:{}, vPZ:{}\n'.format(key, sX, sY, sZ, zrange, vPX, vPY, vPZ)
    f.write(msg)

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # csvFile = open('data.csv', 'wb')
    # csvWriter = csv.writer(csvFile, delimiter=',')
    f = open('myfile.txt', 'w')

    ###no multiple threads
    # cap = cv2.VideoCapture(0)
    # cap.set(3, VIDEO_X_PIXEL)
    # cap.set(4, VIDEO_Y_PIXEL)

    ###multiple threads
    vs = VideoStream('http://127.0.0.1:8080/?action=stream').start()

    # filename = sys.argv[1]
    timeArray = time.localtime()
    otherStyleTime = time.strftime("%Y-%m-%d_%H_%M_%S", timeArray)
    # filename = 'C:\\Users\\PCMYMAC\\Documents\\crazyflie\\Demooutput\\test_'+otherStyleTime + '.avi'
    # ----------------writer-----------------
    # filename = '/home/pi/Demooutput/test_'+otherStyleTime + '.avi'

    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter(filename, fourcc, 25, (VIDEO_X_PIXEL, VIDEO_Y_PIXEL))
    opencv_thread = drawCV(vs)
    opencv_thread.start()

    # with raw(sys.stdin):
    # with nonblocking(sys.stdin):
    with SyncCrazyflie(URI) as scf:
                reset_estimator(scf)
                start_logging(scf)
                # automatic(scf, f)
                countt = 1
                with MotionCommander(scf) as mc:
                    time.sleep(1)

                # cap.release()
                vs.stop()
                # out.release()
                cv2.destroyAllWindows()
                f.close()
                    # sys.exit()
                    # We land when the MotionCommander goes out of scope

