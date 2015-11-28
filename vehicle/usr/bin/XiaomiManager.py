#
# This file handles GoPro commands and holds GoPro state
#
import os
import Queue
import sys
import threading
import time
from pymavlink import mavutil

sys.path.append(os.path.realpath(''))
import app_packet
from GoProConstants import *
import settings
import shotLogger
import struct

logger = shotLogger.logger

class XiaomiManager():
    def __init__(self, shotMgr):
        self.shotMgr = shotMgr
        # This exists because we can't seem to send multiple messages in a stream to the camera.
        # Instead, we'll queue up all our messages and wait for a response before sending the next message
        self.msgQueue = Queue.Queue()
        # when the last message was sent
        self.lastRequestSent = 0.0
        # not recording right now
        self.recording = False

        logger.log("XiaomiManager up and running")

    # handle a call to the "record" command
    def handleToggleRecord(self):
        if self.recording:
            self.execCmd("xiaomi_record_stop.py")
            self.recording = False
            logger.log("Stopped recording")
        else:
            self.execCmd("xiaomi_record_start.py")
            self.recording = True
            logger.log("Started recording")

    def handleTakePicture(self):
        self.execCmd("xiaomi_photo.py")

    def handleReset(self):
        self.execCmd("xiaomi_record_stop.py")
        self.recording = False

    def execCmd(self, cmd):
        os.system(cmd)

    # since the gopro can't handle multiple messages at once, we wait for a response before sending
    # each subsequent message.  This is how we queue up messages
    def queueMsg(self, msg):
        if self.isCameraBusy and time.time() > self.lastRequestSent + 2.0:
            self.isCameraBusy = False
            self.msgQueue = Queue.Queue()
            logger.log("no gopro response delivered before timeout, flushing queue")
            # return current state to resynchronize client
            self.sendState()

        if self.isCameraBusy:
            self.msgQueue.put(msg)
            logger.log("queuing up gopro message.  Queue size is now %d"%(self.msgQueue.qsize()))
        else:
            self.isCameraBusy = True
            self.lastRequestSent = time.time()
            # Need to send False for fix_targeting so our message gets routed to the gimbal
            self.shotMgr.vehicle.send_mavlink(msg, False)
            self.shotMgr.vehicle.flush()
            logger.log("no queue, just sending message")

