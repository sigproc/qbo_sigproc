#!/usr/bin/env python

#Simple Xbox360 joypad test program by fg295

#import ROS dependencies 
import roslib; roslib.load_manifest('qbo_depth_slam')
import rospy
 
#import and init pygame
import sys
import pygame; pygame.init()

#Import ROS message to publish movements
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy

#create the window
window = pygame.display.set_mode((640, 480))
pygame.display.set_caption('Xbox360 Joypad tester by SifuF') 

b_offsetX = 480; b_offsetY = 170
m_offsetX = 310; m_offsetY = 170
d_offsetX = 220; d_offsetY = 325
la_offsetX = 150; la_offsetY = 170
ra_offsetX = 420; ra_offsetY = 325
lb_offsetX = 50; lb_offsetY = 50
rb_offsetX = 600; rb_offsetY = 50
lt_offsetX = 50; lt_offsetY = 120
rt_offsetX = 600; rt_offsetY = 120

#draw a line - see http://www.pygame.org/docs/ref/draw.html for more 
#pygame.draw.line(window, (255, 255, 255), (0, 0), (30, 50))

pygame.draw.circle(window, (0, 0, 100), (b_offsetX-40, b_offsetY), 20)   #X
pygame.draw.circle(window, (0, 100, 0), (b_offsetX, b_offsetY+40), 20)   #A
pygame.draw.circle(window, (100, 0, 0), (b_offsetX+40, b_offsetY), 20)   #B
pygame.draw.circle(window, (100, 100, 0), (b_offsetX, b_offsetY-40), 20) #Y

pygame.draw.circle(window, (100, 100, 100), (d_offsetX-40, d_offsetY), 20)   #left
pygame.draw.circle(window, (100, 100, 100), (d_offsetX, d_offsetY+40), 20)   #down
pygame.draw.circle(window, (100, 100, 100), (d_offsetX+40, d_offsetY), 20)   #right
pygame.draw.circle(window, (100, 100, 100), (d_offsetX, d_offsetY-40), 20) #up

pygame.draw.circle(window, (100, 100, 100), (m_offsetX-40, m_offsetY), 10)   #back
pygame.draw.circle(window, (0, 100, 0), (m_offsetX, m_offsetY), 25)   #centre
pygame.draw.circle(window, (100, 100, 100), (m_offsetX+40, m_offsetY), 10) #start


pygame.draw.circle(window, (100, 100, 100), (la_offsetX, la_offsetY), 40) #lAnalog
pygame.draw.circle(window, (255, 255, 255), (la_offsetX, la_offsetY), 60, 2)
pygame.draw.circle(window, (100, 100, 100), (ra_offsetX, ra_offsetY), 40) #rAnalog
pygame.draw.circle(window, (255, 255, 255), (ra_offsetX, ra_offsetY), 60, 2) 

pygame.draw.circle(window, (100, 100, 100), (lb_offsetX, lb_offsetY), 20) #lb
pygame.draw.circle(window, (100, 100, 100), (rb_offsetX, rb_offsetY), 20) #rb

pygame.draw.circle(window, (255, 255, 255), (lt_offsetX, lt_offsetY), 20) #lt
pygame.draw.circle(window, (255, 255, 255), (rt_offsetX, rt_offsetY), 20) #rt

#draw it to the screen
pygame.display.flip() 


def callback(data):
    
    window.fill((0,0,0))
    
    if data.buttons[0]==1:
        pygame.draw.circle(window, (0, 255, 0), (b_offsetX, b_offsetY+40), 20)   #A
    else:
        pygame.draw.circle(window, (0, 100, 0), (b_offsetX, b_offsetY+40), 20)  

    if data.buttons[1]==1:
        pygame.draw.circle(window, (255, 0, 0), (b_offsetX+40, b_offsetY), 20)   #B
    else:
        pygame.draw.circle(window, (100, 0, 0), (b_offsetX+40, b_offsetY), 20)   

    if data.buttons[2]==1:
        pygame.draw.circle(window, (0, 0, 255), (b_offsetX-40, b_offsetY), 20)   #X
    else:
        pygame.draw.circle(window, (0, 0, 100), (b_offsetX-40, b_offsetY), 20)  

    if data.buttons[3]==1:
        pygame.draw.circle(window, (255, 255, 0), (b_offsetX, b_offsetY-40), 20)   #Y
    else:
        pygame.draw.circle(window, (100, 100, 0), (b_offsetX, b_offsetY-40), 20)  


    if data.buttons[11]==1:
        pygame.draw.circle(window, (255, 255, 255), (d_offsetX-40, d_offsetY), 20)   #left
    else:
        pygame.draw.circle(window, (100, 100, 100), (d_offsetX-40, d_offsetY), 20)    

    if data.buttons[14]==1:
        pygame.draw.circle(window, (255, 255, 255), (d_offsetX, d_offsetY+40), 20)   #down
    else:
        pygame.draw.circle(window, (100, 100, 100), (d_offsetX, d_offsetY+40), 20)      

    if data.buttons[12]==1:
        pygame.draw.circle(window, (255, 255, 255), (d_offsetX+40, d_offsetY), 20)   #right
    else:
        pygame.draw.circle(window, (100, 100, 100), (d_offsetX+40, d_offsetY), 20)  

    if data.buttons[13]==1:
        pygame.draw.circle(window, (255, 255, 255), (d_offsetX, d_offsetY-40), 20) #up
    else:
        pygame.draw.circle(window, (100, 100, 100), (d_offsetX, d_offsetY-40), 20)


    if data.buttons[6]==1:
        pygame.draw.circle(window, (255, 255, 255), (m_offsetX-40, m_offsetY), 10)   #back
    else:
        pygame.draw.circle(window, (100, 100, 100), (m_offsetX-40, m_offsetY), 10)  

    if data.buttons[8]==1:
        pygame.draw.circle(window, (0, 255, 0), (m_offsetX, m_offsetY), 25)   #centre
    else:
        pygame.draw.circle(window, (0, 100, 0), (m_offsetX, m_offsetY), 25)       

    if data.buttons[7]==1:
        pygame.draw.circle(window, (255, 255, 255), (m_offsetX+40, m_offsetY), 10) #start
    else:
        pygame.draw.circle(window, (100, 100, 100), (m_offsetX+40, m_offsetY), 10)
     

    if data.buttons[9]==1:
        pygame.draw.circle(window, (255, 255, 255), (la_offsetX-(int)(20*data.axes[0]), la_offsetY-(int)(20*data.axes[1])), 40) #lAnalog
    else:
        pygame.draw.circle(window, (100, 100, 100), (la_offsetX-(int)(20*data.axes[0]), la_offsetY-(int)(20*data.axes[1])), 40) 

    pygame.draw.circle(window, (255, 255, 255), (la_offsetX, la_offsetY), 60, 2)
    
 
    if data.buttons[10]==1:
        pygame.draw.circle(window, (255, 255, 255), (ra_offsetX-(int)(20*data.axes[3]), ra_offsetY-(int)(20*data.axes[4])), 40) #rAnalog
    else:
        pygame.draw.circle(window, (100, 100, 100), (ra_offsetX-(int)(20*data.axes[3]), ra_offsetY-(int)(20*data.axes[4])), 40) 

    pygame.draw.circle(window, (255, 255, 255), (ra_offsetX, ra_offsetY), 60, 2)



    if data.buttons[4]==1:
        pygame.draw.circle(window, (255, 255, 255), (lb_offsetX, lb_offsetY), 20) #lb
    else:
        pygame.draw.circle(window, (100, 100, 100), (lb_offsetX, lb_offsetY), 20) 
    
    if data.buttons[5]==1:
        pygame.draw.circle(window, (255, 255, 255), (rb_offsetX, rb_offsetY), 20) #lb
    else:
        pygame.draw.circle(window, (100, 100, 100), (rb_offsetX, rb_offsetY), 20) 


    pygame.draw.circle(window, (255, 255, 255), (lt_offsetX, lt_offsetY-(int)(20*data.axes[2])), 20) #lt
    pygame.draw.circle(window, (255, 255, 255), (rt_offsetX, rt_offsetY-(int)(20*data.axes[5])), 20) #rt


    pygame.display.flip() 

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('joypad_test', anonymous=True)

    rospy.Subscriber("/joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
