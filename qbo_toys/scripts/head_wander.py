#!/usr/bin/env python
"""
A fun little script to make Q.bo's head wander randomly.
"""
import random

import rospy
import roslib

roslib.load_manifest('qbo_talk')

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# Yes, the bad spelling is part of the API now :(
from qbo_talk.srv import Text2Speach

UTTERANCES = [
        'I see', 'Gosh', 'What?', 'Hum', 'Oh',
        'Really?', 'Look!',
]

def head_wander():
    rospy.init_node('head_wander')
    joints_pub = rospy.Publisher('/cmd_joints', JointState)
    say = rospy.ServiceProxy('/qbo_talk/festival_say', Text2Speach)

    seq = 0
    while not rospy.is_shutdown():
        seq += 1

        msg = JointState()
        msg.header.seq = seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_footprint'

        msg.name = ['head_pan_joint','head_tilt_joint']
        msg.position = [random.uniform(-0.9,0.9), random.uniform(-0.8,0.3)]

        joints_pub.publish(msg)

        if random.random() < 0.3:
            try:
                say(random.choice(UTTERANCES))
            except rospy.ServiceException:
                # Ignore errors
                pass

        rospy.sleep(2.0)

if __name__ == '__main__':
    try:
        head_wander()
    except rospy.ROSInterruptException:
        pass
