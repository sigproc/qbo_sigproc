# Q.bo USB Xbox 360 Joypad Library Package by fg295: qbo_joypad

This package contains applications to control the Q.bo via the Xbox 360 Joypad  
#### To use:  

First, build the package. In a terminal, navigate to the root of your catkin workspace (eg. ~/cued-masters) and type:
```console
$ catkin_make
```  

Second, install required dependencies (and then re-run catkin_make if it failed before hand):
```console
$ rosdep install qbo_joypad
```  

Third, plug in the Xbox360 controller and hold the 'home' button to power it on (If you are using the wireless controller with USB adapter you must also sync the joypad at this stage by holding the sync button on both adapter and Joypad - Note that the Xbox LED halo will continue to flash after a successful sync, unlike with the xbox console. This is normal)

Finally, to launch the application type:
```console
$ roslaunch qbo_joypad joypad.launch
```  

The program attempts to open the first available Linux joypad (js0) by default. If you would like to use a different device (or your device maps to a different name by default), you can pass the device name in via an argument to the launch file.  
To find your joypad's device name, open a terminal window and type:  
```console
$ ls -l /dev/input
```  

Joypad device names are of the form 'jsX' where X is an integer. Once you've located your device, you can provide the name (the absolute path) to the launchfile as follows:  
```console
$ roslaunch qbo_joypad joypad.launch device:=/dev/input/jsX
```  

So if for example your device was 'js1', you would type:  
```console
$ roslaunch qbo_joypad joypad.launch device:=/dev/input/js1
```  



#### Controls:  
Hold RB + Left thumb stick controls movement  
Right thumb stick controls head  
Right thumb stick click toggles inverted flight controls (Nose turns green when inverted, blue when non-inverted)  
Left and Right triggers control eyelids  
D-pad controls the mouth (LB to turn off mouth LED's)  
ABXY buttons control nose  


## joypad_controller:

Subscribes to the '/joy' topic to recieve joypad state and publishes to '/cmd_vel', '/cmd_joints', '/cmd_nose' and '/cmd_mouth' to control the Q.bo

## joypad_tester:

Subscribes to the '/joy' topic and produes a graphical representation of the Xbox 360 joypad

