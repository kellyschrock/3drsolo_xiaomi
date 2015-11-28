#
# This is the entry point for MavProxy running DroneAPI on the vehicle
# Usage:
# * mavproxy.py
# * module load api
# * api start shotManager.py
#
import os
from os import sys, path
import math
import platform
import Queue
import select
import socket
import struct
import time
import traceback
from droneapi.lib import Location
from droneapi.lib import VehicleMode
from pymavlink import mavutil
sys.path.append(os.path.realpath(''))
import app_packet
import buttonManager
import cable_cam
from GoProConstants import *
import GoProManager
import XiaomiManager
import infiniCable
import location_helpers
import modes
import orbit
import RCRemapper
import selfie
import shotLogger
import shots
from shotManagerConstants import *
# on host systems these files are located here
sys.path.append(os.path.realpath('../../flightcode/stm32'))
import btn_msg
# on host systems these files are located here
sys.path.append(os.path.realpath('../../net/usr/bin'))
import rc_pkt

logger = shotLogger.logger

# cm / s from APM
DEFAULT_WPNAV_SPEED_VALUE = 500.0

class ShotManager():
    def __init__(self):

        # current shot - can be altered by the app through app_vehicle_server
        # see the shotlist in app/shots/shots.py
        self.currentShot = shots.APP_SHOT_NONE
        # FLY mode
        self.currentModeIndex = DEFAULT_APM_MODE
        self.curController = None
        self.rcSkip = 0
        self.isMission = False;
        self.cameraTLogFile = None;
    def Start(self, api):
        logger.log("starting up shotManager 1.3.0")

        # get our vehicle - when running with mavproxy it only knows about one vehicle (for now)
        self.vehicle = api.get_vehicles()[0]

        logger.log("got api vehicle")
        # set up a socket to receive rc inputs from pixrc
        if os.path.exists( "/tmp/shotManager_RCInputs_socket" ):
            os.remove( "/tmp/shotManager_RCInputs_socket" )

        self.rcSock = socket.socket( socket.AF_UNIX, socket.SOCK_DGRAM )
        self.rcSock.bind("/tmp/shotManager_RCInputs_socket")
        self.rcSock.setblocking(0)

        self.remapper = RCRemapper.RCRemapper(self.rcSock, self)

        # set up a TCP server to receive commands from the app
        # Create a TCP/IP socket
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        if platform.system() != 'Darwin':
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            # After 1 second, start KEEPALIVE
            self.server.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)
            # 5 seconds in between keepalive pings
            self.server.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 5)
            # 5 max fails
            self.server.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)

        # Bind the socket to the port
        server_address = ('', SERVER_PORT)

        while True:
            try:
                self.server.bind(server_address)
            except:
                # logger.log("address in use")
                time.sleep(1.0)
                pass
            else:
                break

        logger.log("ready for connections")

        # Listen for incoming connections
        self.server.listen(0)
        self.client = None

        self.inputs = [self.server, self.rcSock]
        self.outputs = []

        self.currentPacket = ""
        self.currentPacketLength = 9999

        self.enableManualGimbalControl()

        self.buttonManager = buttonManager.buttonManager(self)
        self.last_ekf_ok = False
        self.last_armed = False

        # Store off the last received set of RC channels
        # If we have short gaps where we aren't receiving RC, fill them in with this
        self.lastRCData = [RCRemapper.DEFAULT_RC_MID, RCRemapper.DEFAULT_RC_MID, RCRemapper.DEFAULT_RC_MID, RCRemapper.DEFAULT_RC_MID, RCRemapper.DEFAULT_RC_MID, RCRemapper.CHANNEL6_MID, RCRemapper.DEFAULT_RC_MID, RCRemapper.CHANNEL8_MID ]

        # Try to maintain a constant tick rate
        self.timeOfLastTick = time.time()
        # how many ticks have we performed since an RC update?
        self.numTicksSinceRCUpdate = 0

        logger.log("Going to wait for vehicle init")

        while True:
            try:
                self.vehicle.wait_init()
            except:
                exceptStr = traceback.format_exc()
                print exceptStr
                logger.log(exceptStr)
                logger.log("failed to init vehicle")
            else:
                break
        logger.log("vehicle inited")

        self.goproManager = GoProManager.GoProManager(self)
        self.xiaomiManager = XiaomiManager.XiaomiManager(self)

        #check if gimbal is present
        if self.vehicle.mount_status[0] is not None:
            logger.log("Gimbal detected.")
        else:
            logger.log("No gimbal detected.")

        self.vehicle.mode = VehicleMode("LOITER")
        self.lastMode = self.vehicle.mode.name
        # set callback for when modes change
        self.vehicle.add_attribute_observer('mode', self.mode_callback)
        # set callback for when ekf changes
        self.vehicle.add_attribute_observer('ekf_ok', self.ekf_callback)
        # set callback for when armed state changes
        self.vehicle.add_attribute_observer('armed', self.armed_callback)
        # set callback when camera is triggered through a mavlink message
        self.vehicle.add_attribute_observer('camera_trigger', self.camera_trigger_callback)

        # gopro manager callbacks
        self.vehicle.add_attribute_observer('gopro_state', self.goproManager.state_callback)
        self.vehicle.add_attribute_observer('gopro_get_response', self.goproManager.get_response_callback)
        self.vehicle.add_attribute_observer('gopro_set_response', self.goproManager.set_response_callback)

        self.initStreamRates()

    def Run(self):

        while True:
            try:
                #print "in shotManager server loop"
                # handle TCP/RC packets
                # we set a timeout of UPDATE_TIME, so we roughly do this every UPDATE_TIME
                rl, wl, el = select.select( self.inputs, self.outputs, [], UPDATE_TIME )

                for s in rl:
                    if s is self.server:
                        self.connectClient()

                    elif s is self.client:
                        try:
                            data = s.recv( 1 )

                            if data:
                                self.parseClientData(data)

                            else:
                                self.disconnectClient()
                        except socket.error as e:
                            self.disconnectClient()

                    elif s is self.rcSock:
                        self.handleRCSock(s)


                # now handle writes
                for s in wl:
                    try:
                        if self.clientQueue:
                            msg = self.clientQueue.get_nowait()
                    except Queue.Empty:
                        # no messages left, stop checking
                        self.outputs.remove(s)
                    else:
                        try:
                            s.send(msg)
                        except Exception as ex:
                            logger.log("exception on send.  Disconnecting")
                            self.disconnectClient()

                # exceptions
                for s in el:
                    logger.log("exception with " + s.getpeername())
                    if s is self.client:
                        self.disconnectClient()
                    else:
                        self.inputs.remove(s)
                        if s in self.outputs:
                            self.outputs.remove(s)
                        s.close()
                        self.clientQueue = None

                self.handleButtons()
                if time.time() - self.timeOfLastTick > UPDATE_TIME:
                    self.Tick()

            except Exception as ex:
                exceptStr = traceback.format_exc()
                print exceptStr
                strlist = exceptStr.split('\n')

                for i in strlist:
                    logger.log(i)

                if self.client:
                    # send error to app
                    packet = struct.pack('<II%ds' % (len(exceptStr)), app_packet.SOLO_MESSAGE_SHOTMANAGER_ERROR, len(exceptStr), exceptStr)

                    self.client.send(packet)
                    # sleep to make sure the packet goes out (for some reason
                    # setting client.setblocking(1) doesn't work)
                    time.sleep(0.4)
                # stop any stick remapping
                self.remapper.detach()

                if self.vehicle.mode.name == 'GUIDED':
                    self.vehicle.mode = VehicleMode("LOITER")
                # cleanup
                self.server.close()
                self.rcSock.close()
                os._exit(1)


    # I am not sure of the best way to do this TCP server stuff.
    # This seems a bit awkward, carrying state in the class itself.
    def parseClientData( self, data ):
        self.currentPacket += data

        # we always look for 8 bytes to start a SoloPacket
        if len(self.currentPacket) == app_packet.SOLO_MESSAGE_HEADER_LENGTH:
            (self.currentPacketType, self.currentPacketLength) = struct.unpack('<II', self.currentPacket)
            # print "checking received packet..  type is ", self.currentPacketType, self.currentPacketLength

        if len(self.currentPacket) == app_packet.SOLO_MESSAGE_HEADER_LENGTH + self.currentPacketLength:
            value = self.currentPacket[app_packet.SOLO_MESSAGE_HEADER_LENGTH:]
            if self.currentPacketType == app_packet.SOLO_MESSAGE_SET_CURRENT_SHOT:
                shot = struct.unpack('<i', value)[0]
                if shot in shots.SHOT_NAMES:
                    logger.log("[SocketParser]: Trying shot: %s " % shots.SHOT_NAMES[shot])
                    self.enterShot(shot)
                else:
                    logger.log("[SocketParser]: Unknown shot id: %d " % shot)

            elif self.currentPacketType == app_packet.SOLO_MESSAGE_LOCATION:
                if self.curController:
                    (lat, lon, alt) = struct.unpack('<ddf', value)
                    logger.log("location %f, %f, %f" %( lat, lon, alt ) )
                    loc = Location(lat, lon, alt)
                    self.curController.addLocation(loc)

            elif self.currentPacketType == app_packet.SOLO_RECORD_POSITION:
                if self.curController:
                    self.curController.recordLocation()

            elif self.currentPacketType == app_packet.SOLO_CABLE_CAM_OPTIONS:
                if self.curController and self.currentShot == shots.APP_SHOT_CABLECAM:
                    self.curController.handleOptions(value)

            elif self.currentPacketType == app_packet.SOLO_MESSAGE_GET_BUTTON_SETTING:
                # this is a request for the current button mapping.
                # fill in the fields and send it back
                (button, event, shot, APMmode) = struct.unpack('<iiii', value)
                if event == btn_msg.Press:
                    (mappedShot, mappedMode) = self.buttonManager.getFreeButtonMapping(button)
                    logger.log("app requested button mapping for %d"%(button))

                    # send back to the app
                    packet = struct.pack('<IIiiii', app_packet.SOLO_MESSAGE_GET_BUTTON_SETTING, 16, button, event, mappedShot, mappedMode)
                    self.sendPacket(packet)

            # app is trying to map a button
            elif self.currentPacketType == app_packet.SOLO_MESSAGE_SET_BUTTON_SETTING:
                (button, event, shot, APMmode) = struct.unpack('<iiii', value)

                if event == btn_msg.Press:
                    self.buttonManager.setFreeButtonMapping( button, shot, APMmode )
            elif self.currentPacketType == app_packet.SOLO_FOLLOW_OPTIONS:
                if self.curController and self.currentShot == shots.APP_SHOT_FOLLOW:
                    self.curController.handleFollowOptions(value)
            elif self.currentPacketType == app_packet.SOLO_SHOT_OPTIONS:
                if self.curController and self.currentShot != shots.APP_SHOT_CABLECAM:
                    self.curController.handleOptions(value)
            # Gopromanager handles these messages
            elif self.currentPacketType in GoProManager.GOPROMESSAGES:
                self.goproManager.handlePacket( self.currentPacketType, value )
            else:
                logger.log("Got an unknown packet type: %d"%(self.currentPacketType))

            # reset packet parsing
            self.currentPacket = ""



    def handleButtons(self):
        buttonEvent = self.buttonManager.checkButtons()

        if buttonEvent is None:
            return

        button, event = buttonEvent

        if self.currentShot == shots.APP_SHOT_NONE:
            if event == btn_msg.Press:
                if button == btn_msg.ButtonA or button == btn_msg.ButtonB:
                    # see what the button is mapped to
                    (shot, mode) = self.buttonManager.getFreeButtonMapping(button)

                    # only allow entry into these shots if the app is attached
                    allowedShots = [shots.APP_SHOT_ORBIT, shots.APP_SHOT_CABLECAM]

                    if shot in allowedShots and self.client:
                        logger.log("[ButtonManager]: Trying shot: %s" % shots.SHOT_NAMES[shot])
                        self.enterShot(shot)
                    elif mode >= 0:
                        # going around DroneAPI's back until pymavlink gets this change:
                        # https://github.com/mavlink/mavlink/pull/328
                        msg = self.vehicle.message_factory.set_mode_encode(
                                                                                     0,    # target system
                                                                                     1, mode)  #base mode, custom mode


                        # send command to vehicle
                        self.vehicle.send_mavlink(msg)
                        self.vehicle.flush()
                        logger.log("[ButtonManager]: requested a mode change to %d" % (mode))
                elif button == btn_msg.ButtonLoiter:
                    self.notifyPause()
                    self.vehicle.flush()


        else:
            if button == btn_msg.ButtonFly and event == btn_msg.Press:
                if self.currentShot != shots.APP_SHOT_SELFIE or self.vehicle.armed:
                    self.enterShot(shots.APP_SHOT_NONE)
                    logger.log("[ButtonManager]: Exited shot via Fly button")
            else:
                self.curController.handleButton(button, event)

        if button == btn_msg.ButtonCameraClick:
            if event == btn_msg.Press:
                self.xiaomiManager.onButtonPress()
            elif event == btn_msg.Release:
                self.xiaomiManager.onButtonRelease()

    def enterShot(self, shot):
        # check our EKF - if it's bad, reject this shot entry attempt
        if shot != shots.APP_SHOT_NONE and not self.last_ekf_ok:
            shot = shots.APP_SHOT_NONE
            # send back our failure
            packet = struct.pack('<III', app_packet.SOLO_SHOT_ERROR, 4, app_packet.SHOT_ERROR_BAD_EKF)
            self.sendPacket(packet)

            #don't return, fall through to set APP_SHOT_NONE
            logger.log("[enterShot]: Poor EKF, aborting shot entry")

        if self.currentShot != shot:
            if shot == shots.APP_SHOT_NONE:
                self.triggerShot(shot)

            #check if vehicle is OK to enter shot
            if self.vehicle.armed:
                if self.vehicle.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
                    self.triggerShot(shot)
                elif self.vehicle.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
                    logger.log("[enterShot]: Copter needs to be flying before entering shot: %s" % shots.SHOT_NAMES[shot])
                    # unarmed, send an error code to the app
                    packet = struct.pack('<III', app_packet.SOLO_SHOT_ERROR, 4, app_packet.SHOT_ERROR_UNARMED)
                    self.sendPacket(packet)
                elif self.vehicle.system_status == mavutil.mavlink.MAV_STATE_CRITICAL:
                    logger.log("[enterShot]: Copter in MAV_STATE_CRITICAL. Cannot enter shot: %s" % shots.SHOT_NAMES[shot])
                    #critical, send an error code to app
                    packet = struct.pack('<III', app_packet.SOLO_SHOT_ERROR, 4, app_packet.SHOT_ERROR_RTL)
                    self.sendPacket(packet)
                elif self.vehicle.system_status == mavutil.mavlink.MAV_STATE_EMERGENCY:
                    logger.log("[enterShot]: Copter in MAV_STATE_EMERGENCY. Cannot enter shot: %s" % shots.SHOT_NAMES[shot])
                    #emergency, send an error code to app
                    packet = struct.pack('<III', app_packet.SOLO_SHOT_ERROR, 4, app_packet.SHOT_ERROR_RTL)
                    self.sendPacket(packet)
            else:
                logger.log("[enterShot]: Copter needs to be flying before entering shot: %s" % shots.SHOT_NAMES[shot])
                # unarmed, send an error code to the app
                packet = struct.pack('<III', app_packet.SOLO_SHOT_ERROR, 4, app_packet.SHOT_ERROR_UNARMED)
                self.sendPacket(packet)
        else:
            logger.log("[enterShot]: Current shot is: '%s', cant enter same shot, ignoring request." % shots.SHOT_NAMES[shot])


    def triggerShot(self, shot):
        # invalid shot!
        if shot not in shots.SHOT_NAMES:
            logger.log("[triggerShot]: Error, trying to trigger an invalid shot! %d"%(shot))
            return

        self.currentShot = shot
        """
        ok, this is hacky.
        Because of the delay in mode reporting from pixhawk,
        If we change shots while we're in guided to another shot,
        our shot auto-exit protection will kick in and boot us out
        of our new shot.  This prevents that by making sure we're
        not in guided so that protection doesn't kick in.
        """
        if self.lastMode == "GUIDED":
            self.lastMode = "changing"
        if self.currentShot == shots.APP_SHOT_NONE:
            self.curController = None
            self.enableManualGimbalControl()
            # stop any stick remapping
            self.remapper.enableRemapping( False )
            if self.vehicle.mode.name == "GUIDED":
                self.vehicle.mode = VehicleMode("LOITER")
                logger.log("setting vehicle to loiter because we're exiting a shot")
            else:
                logger.log("Didn't set loiter because mode was %s"%(self.vehicle.mode.name))
            # set a decent speed so RTLs don't go super slowly
            # this is needed because nothing resets the DO_CHANGE_SPEED from the shots
            speed = self.getParam( "WPNAV_SPEED", DEFAULT_WPNAV_SPEED_VALUE ) / 100.0
            msg = self.vehicle.message_factory.command_long_encode(
                                                                                 0, 1,    # target system, target component
                                                                                 mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # frame
                                                                                 0,       # confirmation
                                                                                 1, speed, -1, # params 1-3
                                                                                 0.0, 0.0, 0.0, 0.0 ) # params 4-7 (not used)


            # send command to vehicle
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()

        elif self.currentShot == shots.APP_SHOT_SELFIE:
            self.initStreamRates()
            self.curController = selfie.SelfieController(self.vehicle, self)
        elif self.currentShot == shots.APP_SHOT_ORBIT:
            self.initStreamRates()
            self.curController = orbit.OrbitController(self.vehicle, self)
        elif self.currentShot == shots.APP_SHOT_CABLECAM:
            # just for launch!
            # it's possible that our stream rates get overwritten by other GCSes,
            # so we'll set them here.
            self.initStreamRates()
            self.curController = cable_cam.CableCamController(self.vehicle, self)
        #elif self.currentShot == shots.APP_SHOT_INFINICABLE:
            #self.curController = infiniCable.InfiniCableController(self.vehicle)
        elif self.currentShot == shots.APP_SHOT_FOLLOW:
            self.initStreamRates()
            self.curController = orbit.OrbitController(self.vehicle, self)

        #send back a response
        packet = struct.pack('<IIi', app_packet.SOLO_MESSAGE_GET_CURRENT_SHOT, 4, shot)
        self.sendPacket(packet)

        self.buttonManager.setButtonMappings()
        self.buttonManager.setArtooShot(shot)

        logger.log("[triggerShot]: Entered shot: %s" % shots.SHOT_NAMES[shot])

    # allow for manual gimbal control again
    def enableManualGimbalControl(self):
        # back to RC targeting
        msg = self.vehicle.message_factory.mount_configure_encode(
            0, 1,    # target system, target component
            mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,  #mount_mode
            1,  # stabilize roll
            1,  # stabilize pitch
            1,  # stabilize yaw
            )

        self.vehicle.send_mavlink(msg)

        # Always call flush to guarantee that previous writes to the vehicle have taken place
        self.vehicle.flush()

    # send a packet to the app
    def sendPacket(self, pkt):
        if self.client and self.clientQueue:
            self.clientQueue.put(pkt)
            if self.client not in self.outputs:
                self.outputs.append(self.client)

    # returns if the app is connected or not
    def isAppConnected(self):
        return self.client != None

    def mode_callback(self, mode):
        if self.vehicle.mode.name != self.lastMode:
            logger.log("mode_callback:  lastMode is %s, newMode is %s"%(self.lastMode, self.vehicle.mode.name))
            if self.currentShot != shots.APP_SHOT_NONE:
                # looks like somebody switched us out of guided!  Exit our current shot
                if self.lastMode == 'GUIDED':
                    logger.log("detected that we are not in guided, exiting shot!")
                    self.enterShot(shots.APP_SHOT_NONE)
                # alternatively, RTL and LAND always exit shots
                elif self.vehicle.mode.name == 'RTL' or self.vehicle.mode.name == 'LAND':
                    logger.log("RTL or land exited shot!")
                    self.enterShot(shots.APP_SHOT_NONE)

            self.lastMode = self.vehicle.mode.name
            logger.log("vehicle changed to mode %s"%self.lastMode)

            # When user is in mission mode, set the camera to photo and open the camera_msgs tlog file
            if self.lastMode == 'AUTO':
                self.goproManager.sendGoProCommand(mavutil.mavlink.GOPRO_COMMAND_CAPTURE_MODE, (CAPTURE_MODE_PHOTO, 0 ,0 , 0))
                self.cameraTLogFile = open('/log/camera_msgs.tlog', 'wb')
                self.isMission = True
            else:
                if (self.cameraTLogFile is not None):
                    self.cameraTLogFile.close()
                    self.cameraTLogFile = None
                self.isMission = False

            # don't do the following for guided, since we're in a shot
            if self.lastMode == 'GUIDED':
                return

            modeIndex = modes.getAPMModeIndexFromName( self.lastMode, self.vehicle)

            if modeIndex < 0:
                logger.log("couldn't find this mode index: %s"%self.lastMode)
                return

            if self.currentShot == shots.APP_SHOT_NONE:
                self.buttonManager.setArtooShot( -1, modeIndex )
                self.currentModeIndex = modeIndex

    def ekf_callback(self, ok):
        if self.vehicle.ekf_ok != self.last_ekf_ok:
            self.last_ekf_ok = self.vehicle.ekf_ok
            logger.log("EKF status changed to %d"%(self.vehicle.ekf_ok))
            self.buttonManager.setButtonMappings()

    def armed_callback(self, ok):
        if self.vehicle.armed != self.last_armed:
            self.last_armed = self.vehicle.armed
            logger.log("armed status changed to %d"%(self.vehicle.armed))
            self.buttonManager.setButtonMappings()

            if not self.vehicle.armed:
                self.enterShot(shots.APP_SHOT_NONE)

    def initStreamRates(self):
        STREAM_RATES = {
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS: 2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: 10,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2: 10,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3: 2,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION: 10,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS: 2,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER: 3,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS: 5,
        }

        for stream, rate in STREAM_RATES.items():
            msg = self.vehicle.message_factory.request_data_stream_encode(
                                                            0, 1,    # target system, target component
                                                            stream,        # requested stream id
                                                            rate,    # rate
                                                            1       # start it
                                                            )

            self.vehicle.send_mavlink(msg)

        self.vehicle.flush()

    def notifyPause(self, inShot=0):
        '''notify the autopilot that we would like to pause'''

        msg = self.vehicle.message_factory.command_long_encode(
            0,                                            # target system
            1,                                            # target component
            mavutil.mavlink.MAV_CMD_SOLO_BTN_PAUSE_CLICK, # frame
            0,                                            # confirmation
            int(inShot),                                  # param 1: 1 if Solo is in a shot mode, 0 otherwise
            0, 0, 0, 0, 0, 0)                             # params 2-7 (not used)

        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    # establishes a connection to a Solo app
    def connectClient(self):
        if self.client:
            # This might not be the cleanest way to do this.
            # The alternative would be closing self.server when a client connects
            # But that would require reopening it, and I've seen issues doing that where it takes longer than I would like.
            client, client_address = self.server.accept()
            logger.log("already connected to a client - rejecting " + str(client_address))
            client.close()
        else:
            # establish a new client connection
            self.client, client_address = self.server.accept()
            logger.log("connected to " + str(client_address))
            self.client.setblocking(0)
            self.inputs.append(self.client)

            # queue for outgoing messages
            self.clientQueue = Queue.Queue()
            self.buttonManager.setButtonMappings()
            self.goproManager.sendState()

    def disconnectClient(self):
        logger.log("closing client connection - data is nil")
        if self.client:
            if self.client in self.outputs:
                self.outputs.remove(self.client)
            if self.client in self.inputs:
                self.inputs.remove(self.client)
            self.client.close()
            self.client = None
        self.clientQueue = None
        self.buttonManager.setButtonMappings()
        self.enterShot(shots.APP_SHOT_NONE)

    # This fetches and returns the value of the parameter matching the given name
    # If the parameter is not found, returns the given default value instead
    def getParam(self, name, default=None):
        """
        This is the desired version, but it fails because MPParameters.wait_valid is infinite-looping
        try:
            return self.vehicle.parameters[name]
        except KeyError:
            return default
        """
        # Instead, we must use this version
        # Unfortunately, for now we are grabbing this internal dict.
        # This will be fixed when we ditch MavProxy
        paramDict = self.vehicle.parameters._MPParameters__module.mav_param
        return paramDict.get( name, default )

    # we call this at our UPDATE_RATE
    # drives the shots as well as anything else timing-dependent
    def Tick(self):
        #print "time since last tick %f"%( time.time() - self.timeOfLastTick )
        self.timeOfLastTick = time.time()
        self.numTicksSinceRCUpdate += 1

        # Only pass RC input in if we're in a shot
        if self.currentShot != shots.APP_SHOT_NONE and self.curController:
            # only forward RC data to pixRC if self.numTicksSinceRCUpdate is small (meaning RC is new)
            channels = self.remapper.remap(self.lastRCData, self.numTicksSinceRCUpdate < 2)
            self.curController.handleRCs(channels)

    # This is called whenever we have data on the rc socket, which should be
    # at 50 hz, barring any drops
    # We remap/normalize the RC and then store it away for use in Tick()
    def handleRCSock(self, sock):
        datagram = sock.recv( 1000 )

        if datagram == None or len(datagram) != rc_pkt.LENGTH:
            return

        self.lastRCData = datagram
        self.numTicksSinceRCUpdate = 0

    def camera_trigger_callback(self, msg):
        if self.isMission:
            msg = self.vehicle.camera_trigger_msg
            self.cameraTLogFile.write(struct.pack('>q', time.time() * 10000000))
            self.cameraTLogFile.write(msg.get_msgbuf())
            self.cameraTLogFile.flush()
            self.goproManager.handleRecordCommand(CAPTURE_MODE_PHOTO, RECORD_COMMAND_START)
