# Q.bo USB Xbox 360 Joypad Library Package: qbo_joypad

This package contains applications to control the Q.bo via Xbox 360 Joypad
To run the package, type:  
roslaunch qbo_joypad joypad.launch

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

