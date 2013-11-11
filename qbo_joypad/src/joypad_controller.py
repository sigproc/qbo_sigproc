#!/usr/bin/env python

#Xbox360 joypad controller for qbo by fg295

#import ROS dependencies 
import roslib; roslib.load_manifest('qbo_joypad')
import rospy
import random
 
#Import ROS message to publish movements
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from qbo_joypad.msg import Mouth
from qbo_joypad.msg import Nose
from std_msgs.msg import Header

#Import other libraries
import time
import sys

inverted='false'
rp_latch='false'

#Publish a message to move the robot 
def move(publisher,linear,ang):
    speed_command=Twist()
    speed_command.linear.x=linear
    speed_command.linear.y=0
    speed_command.linear.z=0
    speed_command.angular.x=0
    speed_command.angular.y=0
    speed_command.angular.z=ang
    publisher.publish(speed_command)

#Publish a message to move the head and eyelids
def head(publisher,horizontal,vertical,left,right):
    head_command=JointState()
    head_command.name = ['head_pan_joint','head_tilt_joint','left_eyelid_joint','right_eyelid_joint']
    head_command.position = [horizontal, vertical, left, right]
    publisher.publish(head_command)

def mouth(publisher, emotion):
    mouth_command=Mouth()
    
    if emotion == 'happy':
        mouth_command.mouthImage=[0,0,0,0,0,1,0,0,0,1,0,1,1,1,0,0,0,0,0,0]
    
    elif emotion == 'sad':
        mouth_command.mouthImage=[0,0,0,0,0,0,1,1,1,0,1,0,0,0,1,0,0,0,0,0]

    elif emotion == 'calm':
        mouth_command.mouthImage=[0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0]

    elif emotion == 'shocked':
        mouth_command.mouthImage=[1,1,1,1,1,1,0,0,0,1,1,0,0,0,1,1,1,1,1,1]

    else:
        mouth_command.mouthImage=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
   
    publisher.publish(mouth_command)

def nose(publisher, nose_color):
    nose_command=Nose()
    nose_header=Header()    
    nose_command.header=nose_header
    
    if nose_color == 'cyan':
        nose_command.color=6
    
    elif nose_color == 'green':
        nose_command.color=4

    elif nose_color == 'blue':
        nose_command.color=2
    
    else:
        nose_command.color=0
    
    publisher.publish(nose_command)


#Called when data arrives on the '/Joy' topic
def callback(data):
    
    global inverted
    global rp_latch
    #Init the publishers. 'pub' for movement, 'joints_pub' for head/eyelids.
    pub=rospy.Publisher('/cmd_vel', Twist)
    joints_pub = rospy.Publisher('/cmd_joints', JointState)
    mouth_pub = rospy.Publisher('/cmd_mouth', Mouth)
    nose_pub = rospy.Publisher('/cmd_nose', Nose)
 
    #Complete button mapping. 'buttons' can be either 1 or 0. 'axes' ranges from -1 to 1.
    
    #Wireless Joypad
    #A               data.buttons[0]==1
    #B               data.buttons[1]==1
    #X               data.buttons[2]==1
    #Y               data.buttons[3]==1
    #left            data.buttons[11]==1
    #down            data.buttons[14]==1
    #right           data.buttons[12]==1
    #up              data.buttons[13]==1
    #back            data.buttons[6]==1
    #centre          data.buttons[8]==1
    #start           data.buttons[7]==1
    #lAnalog button  data.buttons[9]==1
    #rAnalog button  data.buttons[10]==1
    #lb              data.buttons[4]==1
    #rb              data.buttons[5]==1
    #lt              data.axes[2] 
    #rt              data.axes[5]
    #lAnalog(x, y)   data.axes[1], data.axes[0]
    #rAnalog(x, y)   data.axes[3], data.axes[4]

    #wired Joypad
    #A               data.buttons[0]==1
    #B               data.buttons[1]==1
    #X               data.buttons[2]==1
    #Y               data.buttons[3]==1
    #back            data.buttons[6]==1
    #centre          data.buttons[8]==1
    #start           data.buttons[7]==1
    #lAnalog button  data.buttons[9]==1
    #rAnalog button  data.buttons[10]==1
    #lb              data.buttons[4]==1
    #rb              data.buttons[5]==1
    #lt              data.axes[2] 
    #rt              data.axes[5]
    #lAnalog(x, y)   data.axes[1], data.axes[0]
    #rAnalog(x, y)   data.axes[3], data.axes[4]
    #left and right  data.axes[6]         
    #up and down     data.axes[7]       

    #No action unless rb is held (safety button).
    if data.buttons[5]==1:   
        move(pub,data.axes[1],data.axes[0])                                  
        
    #if rb is released, stop the robot
    else:
        move(pub,0,0) 


    if(inverted=='true'):
        head(joints_pub,data.axes[3],data.axes[4],data.axes[2],data.axes[5])
        
    else:
        head(joints_pub,data.axes[3],-data.axes[4],data.axes[2],data.axes[5])
    

    #Mouth
    #We now need to check for wired vs wireless controllers
    
    if len(data.buttons) < 12:  
        #Wired
        if data.axes[7]>0.5:
            mouth(mouth_pub, 'happy') 

        elif data.axes[7]<-0.5:
            mouth(mouth_pub, 'sad') 

        if data.axes[6]>0.5:
            mouth(mouth_pub, 'calm') 

        elif data.axes[6]<-0.5:
           mouth(mouth_pub, 'shocked')

        if data.buttons[4]==1:
           mouth(mouth_pub, 'off')
  
    
    else:    
        #wireless
        if data.buttons[13]==1:
            mouth(mouth_pub, 'happy') 

        if data.buttons[14]==1:
            mouth(mouth_pub, 'sad') 

        if data.buttons[11]==1:
            mouth(mouth_pub, 'calm') 

        if data.buttons[12]==1:
           mouth(mouth_pub, 'shocked')

        if data.buttons[4]==1:
           mouth(mouth_pub, 'off')


   #Nose
    if data.buttons[0]==1:
        nose(nose_pub, 'green') 

    if data.buttons[2]==1:
        nose(nose_pub, 'blue') 

    if data.buttons[1]==1:
        nose(nose_pub, 'cyan') 

    if data.buttons[3]==1:
        nose(nose_pub, 'off')

    
    #Invert head
    if data.buttons[10]==1 and rp_latch=='false':
        
        if inverted=='true':
            inverted='false'
            nose(nose_pub, 'blue')
        else: 
            inverted='true'
            nose(nose_pub, 'green')
        
        rp_latch='true'
    
    elif not data.buttons[10]==1:
        rp_latch='false'

  

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('joypad_controller', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
