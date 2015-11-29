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

CMD_RECORD_START = 513
CMD_RECORD_STOP = 514
CMD_PHOTO = 769

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

        # Long press?
        if diff > 1:
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
        return self._sendMsg(CMD_RECORD_START)

    def _xiaomiRecordStop(self):
        return self._sendMsg(CMD_RECORD_STOP)

    def _xiaomiPhoto(self):
        return self._sendMsg(CMD_PHOTO)

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


