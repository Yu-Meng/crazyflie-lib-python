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
import sys
import math
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from threading import Thread
import json

import time
import traceback

from socket import *
import struct

##LED
from raspledstrip.ledstrip import *
from raspledstrip.color import *

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

##LED
led = LEDStrip(4, use_py_spi=True)

def init_hover():
    mc.up(0.4, velocity=0.2)
    time.sleep(1)

nothingFlag = False
def selfcontrol():
    action_status = 0
    timing_status = 0
    timeOut = 5
    nothingStart = 0
    nothingEnd = 0
    print("start fly")
    global objectCX
    global objectCY
    global nothingFlag
    nothingFlag = False
    while True:
        # print(objectCX, objectCY)
        if face_tracking_mode:

            if action_status == 0:
                if objectCX > 0 and objectCX < VIDEO_X_PIXEL * 0.35:
                    if action_status == 0:
                        mc.start_turn_left(rate=30)
                        print('[Flight Command]: mc.start_turn_left(rate=30)')
                        action_status = 1
                if objectCX > VIDEO_X_PIXEL * 0.65 and objectCX <= VIDEO_X_PIXEL:
                    mc.start_turn_right(rate=30)
                    print('[Flight Command]: mc.start_turn_right(rate=30)')
                    action_status = 2
            elif (objectCX >= VIDEO_X_PIXEL * 0.35 and objectCX <= VIDEO_X_PIXEL * 0.65) or (objectCX < 0):
                mc.stop()
                print('[Flight Command]: mc.stop()')
                action_status = 0
            if action_status == 1:
                led.set(0, Color(0, 0, 255, 1))
                led.update()
                led.set(1, Color(0, 0, 255, 1))
                led.update()
            if action_status == 2:
                led.set(2, Color(0, 0, 255, 1))
                led.update()
                led.set(3, Color(0, 0, 255, 1))
                led.update()
            if action_status == 0:
                led.all_off()
                led.update()

            objectCX = objectCY = -1

        if nothingFlag:
            # time.sleep(1)
            mc.down(0.8, velocity=0.2)
            print('[Flight Command]: mc.down(0.4, velocity=0.2)')
            break
        time.sleep(0.1)



######FlightControl Thread########

BBOX_FORMAT = "!QHHHHHH"

BBOX_SIZE = struct.calcsize(BBOX_FORMAT)

class FlightControlThread:

    def __init__(self):

        s = socket(AF_INET, SOCK_DGRAM)
        s.bind((HOST, PORT))
        s.settimeout(6)
        self.serverstream = s

        s = socket(AF_INET, SOCK_DGRAM)
        self.clientstream = s

        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        self.t = Thread(target=self.update, args=())
        self.t.start()
        return self

    def update(self):
        global objectCX
        global objectCY
        global nothingFlag
        # keep looping infinitely until the thread is stopped
        while not self.stopped:
            try:
                ret, addr = self.serverstream.recvfrom(BBOX_SIZE)
                nothingFlag = False
                frame_id, fw, fh, x, y, w, h = struct.unpack(BBOX_FORMAT, ret)
                objectCX = x + w / 2
                objectCY = y + h / 2
                # j_str = json.dumps({"fid": frame_id, "bbox_x": x, "bbox_y": y, "bbox_w": w, "bbox_h": h})
                # self.clientstream.send_to(j_str, (OHOST, OPORT))

                # print("-------RECEIVE--------", frame_id,fw, fh, x, y, w, h)
            except timeout:
                # print("hihihi")
                nothingFlag = True



    def stop(self):
        # indicate that the thread should be stopped

        addrs = getaddrinfo(gethostname(), None)
        # print(addrs)
        j_str = json.dumps({"Type": "flight control", "status": 0, "message": "Flight stop", "src": [item[4][0] for item in addrs if ':' not in item[4][0]][0]})
        self.clientstream.sendto(j_str.encode(), (OHOST, OPORT))
        self.stopped = True
        self.t.join()





if __name__ == '__main__':

    if len(sys.argv) < 5:
        print("Usage: ./program ServerHost ServerPort OutputHost OutputPort")
        exit()

    HOST, PORT = sys.argv[1], int(sys.argv[2])#'192.168.10.1', 25000
    OHOST, OPORT = sys.argv[3], int(sys.argv[4])#'192.168.10.1', 25000

    print("\n\n[INFO] starting flight control server...")
    fcs = FlightControlThread().start()


    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    timeArray = time.localtime()
    otherStyleTime = time.strftime("%Y-%m-%d_%H_%M_%S", timeArray)
    try:
        with SyncCrazyflie(URI) as scf:

                    countt = 1
                    with MotionCommander(scf) as mc:
                        init_hover()
                        time.sleep(1)
                        selfcontrol()


    except KeyboardInterrupt as e:
        print(str(e), traceback.format_exc())

    except Exception as e:
        print(str(e), traceback.format_exc())

    finally:
        print("\n\n[INFO] stopped flight control server...")
        fcs.stop()
