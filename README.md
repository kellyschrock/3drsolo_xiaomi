# Xiaomi Yi Camera control from Solo

After getting a Feiyu-Tech Mini3D gimbal to work on the Solo, I thought it would be kind of cool to be able to control the camera itself from the "paddle" control on the controller to start and stop video recording. So that's what this is about.

## Basic Idea

One cool feature of the Xiaomi Yi camera is that it exposes a service for controlling it over a network. (This is the service the Yi app uses to change settings, record, take pictures, etc.)

What we're interested in here is the ability to start and stop recording, and take pictures when using the "paddle" control on the Solo's transmitter. To do this, we need the following:

*   To communicate with the camera from the Solo
*   To have the Solo recognize when we're punching the appropriate buttons on the controller

When I click the paddle (and by "click", I mean "push straight down on it so it clicks"), I want to start recording. Click it again, and it stops recording. If I long-click the paddle, take a still picture. 

***Note***: It's normally considered a very bad idea to fly around with a mounted action camera with its wifi turned on. It can cause interference with the transmitter signal. However, in the Solo's case, the connection between the transmitter and the Solo *is* a wifi connection. I'm not an RF engineer, but I don't expect a device on the same AP as the Solo and transmitter (and the app) to cause any problems. In any case, I haven't seen any evidence of interference in my testing.

## Camera Setup

Out of the box, the Yi is in AP mode. For this, the camera needs to be in *station* mode in order to connect to an existing wifi network. It also needs to have a static IP address so the Solo can communicate with it.

The `sdcard.zip` file contains a bunch of files you can unpack onto the root of the SD card on the camera. Do this, and edit the `wpa_supplicant.conf` file to reference your SoloLink_xxxx network, wifi password, etc.

Unmount your camera (so the files actually get saved to the SD card), unplug the camera from USB, and it will power off.

The next time you start the camera, it's going to try to connect to your Solo's wifi network. So the Solo and TX need to be up and running before you power up the camera. Ideally, all the scripts that run at startup would only run when you turn wifi on. 

### Camera Startup
Power the camera up, and wait for the power-button LED on the front to stop flashing. You'll have 30 seconds to turn the camera's wifi on. You'll hear a short beep, at which point the camera starts working on getting itself into "station" mode. Then you'll hear a long beep, indicating that it's in station mode and connected.

**NOTE:** This doesn't really make any permanent modifications to the camera. To get normal behavior, rename `autoexec.ash` to something else and reboot the camera. It will come up in AP mode like it normally does, and you can use the Xiaomi app with it like you normally would. 

#### CREDITS: 

All of this is the work of some reverse-engineering Xiaomi Yi geniuses. The original code I based my code on is from them:

*   https://dashcamtalk.com/forum/threads/camera-wifi-in-client-mode-working-ash-script-inside.12239/
/home/kellys/work/solo/xiaomi/wifi_station_mode
*   https://github.com/kerenmac/Xiaomi-Yi
*   https://github.com/deltaflyer4747/Xiaomi_Yi

#### Test the connection
Having gotten this far, you can test the camera from your PC by doing this:

*   Power up your Solo and controller.
*   Join your SoloLink_xxxx AP.
*   Power up the camera, turn its wifi on, and wait around for it to connect.
*   Run `xiaomi_record_start.py`.

The camera should start recording. 

## Solo Setup

**Disclaimer:** I don't know if this voids your warranty or not, but if it does, don't say I didn't warn you. It probably will. I accept no responsibility whatsoever for bad things that happen to you as a result of reading anything in this document, etc. If you're not totally comfortable with the idea of screwing around with the code that runs on your Solo, ***DO NOT PROCEED*** past this point. Seriously. 

The first thing to do is connect to the SoloLink_xxxx AP with your computer, and ssh into the Solo. See http://dev.3dr.com/ for some good guidance on getting around on the Solo from a terminal.

Once you're in, run these commands:

```
root@3dr_solo# cd /usr/bin
root@3dr_solo# cp -v shotManager.py shotManager.save.py
```

Now, you can safely replace the existing `shotManager.py` script with the updated one that talks to the Xiaomi camera.

The only changes to `shotManager.py` are as follows:

* `import XiaomiManager` (just below "import GoProManager")
* `self.xiaomiManager = XiaomiManager.XiaomiManager(self)` (just below "self.goproManager = GoProManager.GoProManager(self)")

...and at the bottom of `handleButtons(self):`
```
        if button == btn_msg.ButtonCameraClick:
            if event == btn_msg.Press:
                self.xiaomiManager.onButtonPress()
            elif event == btn_msg.Release:
                self.xiaomiManager.onButtonRelease()
```

### Changing files

Use scp or similar to push the `shotManager.py` and `XiaomiManager.py` scripts to `/usr/bin` on the Solo. From the root of this project's directory, do this:

```
$ cd vehicle/usr/bin
$ scp *Manager.py root@10.1.1.10:/usr/bin
```

Once you've copied the files to the Solo, reboot it.

Make sure the Solo, controller, and the Camera are all up and running and connected to the SoloLink_xxxx AP.

Click the paddle on the Solo's controller. If everything's working, you should see the Yi's front light flashing, indicating that it's recording. Press the paddle again, and it should stop. Long-click (for more than 1 second) and release. It should take a still picture.

## Using it

Click down on the paddle to toggle recording. To take a still picture, hold the paddle down for more than 1 second. (This won't work if you're already recording.)

One somewhat-cool thing about this is that the Xiaomi camera doesn't actually have to be mounted to the Solo in order to work like this. If, for example, already have the Solo gimbal and a GoPro on your Solo but you need (for whatever reason) to make a video of something else in sync with the GoPro's videos, the Xiaomi will start and stop recording at the same time your Solo-mounted GoPro does.

That's about it.


