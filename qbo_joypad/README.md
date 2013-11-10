# Q.bo USB Xbox 360 Joypad Library Package by fg295: qbo_joypad

This package contains applications to control the Q.bo via Xbox 360 Joypad
To use:  

First, build the package. In a terminal, navigate to the root of your catkin workspace (eg. ~/cued-masters) and type:
```console
$ catkin_make
```  

Second, install required dependencies:
```console
$ rosdep install qbo_joypad
```  

Third, plug in the Xbox360 controller and hold the 'home' button to power it on (If you are using the wireless controller with USB adapter you must also sync the joypad at this stage by holding the sync button on both adapter and Joypad - Note that the Xbox LED halo will continue to flash after a successful sync, unlike the xbox. This is normal)

Finally, to launch the application type:
```console
$ roslaunch qbo_joypad joypad.launch
```  

Controls:  
Hold RB + Left thumb stick controls movement  
Right thumb stick controls head  
Left and Right triggers control eyelids  
D-pad controls the mouth (LB to turn off mouth LED's)  
ABXY buttons control nose  


## joypad_controller:

Subscribes to the '/joy' topic to recieve joypad state and publishes to '/cmd_vel', '/cmd_joints', '/cmd_nose' and '/cmd_mouth' to control the Q.bo

## joypad_tester:

Subscribes to the '/joy' topic and produes a graphical representation of the Xbox 360 joypad

