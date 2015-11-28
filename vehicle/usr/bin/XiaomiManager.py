#
# This file handles basic Xiaomi interaction
#
import os
import re
import sys
import time
import socket

sys.path.append(os.path.realpath(''))
# import settings
import shotLogger

logger = shotLogger.logger

camaddr = "10.1.1.12"
camport = 7878

class XiaomiManager():
    def __init__(self, shotMgr):
        self.shotMgr = shotMgr
        # not recording right now
        self.recording = False

        self.srv = None
        self.token = None
        self.buttonPressTime = 0.0

        logger.log("XiaomiManager up and running")

    def onButtonPress(self):
        self.buttonPressTime = time.time()
        logger.log("onButtonPress(): time=%f" % self.buttonPressTime)

    def onButtonRelease(self):
        now = time.time()
        diff = (now - self.buttonPressTime)

        logger.log("onButtonRelease(): diff=%f" % diff)

        if diff > 5:
            # really long press
            self.handleReset()
        elif diff > 2:
            if self.recording:
                logger.log("Can't take picture, recording")
            else:
                self.handleTakePicture()
        else:
            # Just a click
            self.handleToggleRecord()

    # handle a call to the "record" command
    def handleToggleRecord(self):
        if self.recording:
            self._xiaomiRecordStop()
            self.recording = False
        else:
            self.recording = self._xiaomiRecordStart()

        logger.log("Recording: %s" % self.recording)

    def handleTakePicture(self):
        self._xiaomiPhoto()

    def handleReset(self):
        self._xiaomiRecordStop()
        self.recording = False

    def _xiaomiRecordStart(self):
        return self._sendMsg(513)

    def _xiaomiRecordStop(self):
        return self._sendMsg(514)

    def _xiaomiPhoto(self):
        return self._sendMsg(769)

    def _sendMsg(self, msgid):
        logger.log("_sendMsg(): msgid=%d" % msgid)

        try:
            srv = self._getSrv()
            token  = self._getToken(srv)

            if srv is None:
                logger.log("Could not get socket")
            else:
                if token is None:
                    logger.log("Could not get token")
                else:
                    tosend = '{"msg_id":%d,"token":%s}' %(msgid, token)

                    srv.send(tosend)
                    srv.recv(512)

                    return True

        except socket.error as msg:
            logger.log("Socket error: %s" % msg)
            self.token = None
            self.srv = None

        return False

    def _getSrv(self):
        if self.srv is None:
            try:
                self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.srv.settimeout(5)
                self.srv.connect((camaddr, camport))
            except socket.error as msg:
                logger.log("Socket error: %s" % msg)
                self.srv = None

        return self.srv

    def _getToken(self, srv):
        if self.token is None:
            token = None

            if srv is None:
                pass
            else:
                srv.send('{"msg_id":257,"token":0}')
                data = srv.recv(512)

                if "rval" in data:
                    token = re.findall('"param": (.+) }',data)[0]
                else:
                    data = srv.recv(512)
                    token = re.findall('"param": (.+) }',data)[0]

                self.token = token

        return self.token

    # # since the gopro can't handle multiple messages at once, we wait for a response before sending
    # # each subsequent message.  This is how we queue up messages
    # def queueMsg(self, msg):
    #     if self.isCameraBusy and time.time() > self.lastRequestSent + 2.0:
    #         self.isCameraBusy = False
    #         self.msgQueue = Queue.Queue()
    #         logger.log("no gopro response delivered before timeout, flushing queue")
    #         # return current state to resynchronize client
    #         self.sendState()

    #     if self.isCameraBusy:
    #         self.msgQueue.put(msg)
    #         logger.log("queuing up gopro message.  Queue size is now %d"%(self.msgQueue.qsize()))
    #     else:
    #         self.isCameraBusy = True
    #         self.lastRequestSent = time.time()
    #         # Need to send False for fix_targeting so our message gets routed to the gimbal
    #         self.shotMgr.vehicle.send_mavlink(msg, False)
    #         self.shotMgr.vehicle.flush()
    #         logger.log("no queue, just sending message")

