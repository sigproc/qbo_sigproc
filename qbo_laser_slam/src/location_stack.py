#!/usr/bin/env python

#A node for saving visited locations by fg295. It is a two step process:
#1. Place the current location (within the map frame) on to the stack.
#2. Send nav goals to previously stored locations

#import Wx dependencies 
import wx
import os

#import ROS dependencies 
import roslib; roslib.load_manifest('qbo_laser_slam')
import rospy
import tf
 
#Import ROS messages to prepare nav goals
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.msg import tfMessage
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from actionlib_msgs.msg import GoalID


#Import ROS message to control Qbo
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from qbo_joypad.msg import Mouth
from qbo_joypad.msg import Nose

#Import other libraries
import time
import sys

#global
location=[]

#def callback(data):
    #print data.transforms[0].header.frame_id
    #print data.transforms[0].child_frame_id

def save_location():
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    start=rospy.Time()
    escape=0
    while escape==0:
        try:
           (trans,rot) = listener.lookupTransform("map", "base_link", start)
           escape=1
    
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            escape=0
            print 'waiting for transform'
            continue

        rate.sleep()
    
    print trans 
    print rot
    global location

    robot_pose=Pose()
    robot_pose.position=Point()
    robot_pose.position.x=trans[0]
    robot_pose.position.y=trans[1]
    robot_pose.position.z=trans[2]

    robot_pose.orientation=Quaternion()
    robot_pose.orientation.x=rot[0]
    robot_pose.orientation.y=rot[1]
    robot_pose.orientation.z=rot[2]
    robot_pose.orientation.w=rot[3]

    location.append(robot_pose)

    print "Saving robot pose # "
    print len(location)
    print robot_pose
    

def publish_goal():
    global location
    rate = rospy.Rate(10.0)
    pub=rospy.Publisher('/move_base_simple/goal', PoseStamped)
    rate.sleep()  
    header=Header()
    #header.seq=0
    header.stamp=rospy.Time.now()
    header.frame_id='map'

    pose=Pose()
    print "publishing robot pose #"
    print len(location)
    pose=location.pop()

    poseStamped=PoseStamped()
    poseStamped.header=header
    poseStamped.pose=pose  
    
    rate.sleep()  
    pub.publish(poseStamped)  
    print poseStamped

def cancel_goal():
    pubc=rospy.Publisher('/move_base/cancel', GoalID)
    goal=GoalID()
    pubc.publish(goal)


def listener():
    rospy.init_node('location_stack', anonymous=True)
    #rospy.Subscriber("/tf", tfMessage, callback)
    app = wx.App(False)
    frame = MainWindow(None, "Qbo Location Saver")
    app.MainLoop()
    rospy.spin()

class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        self.dirname=''

        wx.Frame.__init__(self, parent, title=title, size=(200,-1))

        

        self.control = wx.TextCtrl(self, style=wx.TE_MULTILINE|wx.EXPAND,size=(300, -1))
        self.CreateStatusBar() # A Statusbar in the bottom of the window

        self.sizer2 = wx.BoxSizer(wx.HORIZONTAL)
        self.buttonSave = wx.Button(self, -1, "Save Location")
        self.buttonSend = wx.Button(self, -1, "Send Location")
        self.sizer2.Add(self.buttonSave, 1, wx.EXPAND)
        self.sizer2.Add(self.buttonSend, 1, wx.EXPAND)
        self.Bind(wx.EVT_BUTTON, self.OnClickSave,self.buttonSave)
        self.Bind(wx.EVT_BUTTON, self.OnClickSend,self.buttonSend)

        self.sizer3 = wx.BoxSizer(wx.HORIZONTAL)
        self.buttonCancel = wx.Button(self, -1, "Cancel Nav Goal")
        self.sizer3.Add(self.buttonCancel, 1, wx.EXPAND)
        self.Bind(wx.EVT_BUTTON, self.OnClickCancel,self.buttonCancel)

        # Use some sizers to see layout options
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.sizer2, 0, wx.EXPAND)        
        self.sizer.Add(self.sizer3, 0, wx.EXPAND)     
        self.sizer.Add(self.control, 1, wx.EXPAND)

        #Layout sizers
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)
        self.Show()


    def OnClickSave(self,e):
        self.control.AppendText("Stored current location! %d\n" %e.GetId())
        save_location()
    
    
    def OnClickSend(self,e):    
        self.control.AppendText("Published nav goal! %d\n" %e.GetId())
        publish_goal()    
    
    def OnClickCancel(self,e): 
        self.control.Clear()   
        self.control.AppendText("Canceled nav goal! %d\n" %e.GetId())
        cancel_goal()
       

        
if __name__ == '__main__':
    listener()
