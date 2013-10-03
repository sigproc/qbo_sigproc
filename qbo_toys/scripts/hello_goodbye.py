#!/usr/bin/env python
"""
A fun little script to make Q.bo's head wander randomly.
"""
import random

import rospy
import roslib

roslib.load_manifest('qbo_talk')
roslib.load_manifest('qbo_face_msgs')

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from qbo_face_msgs.msg import FacePosAndDist

# Yes, the bad spelling is part of the API now :(
from qbo_talk.srv import Text2Speach

import numpy as np

HISTORY_SIZE=20

STATE = {
        'cursor': 0,
        'found_face': False,
        'detections': np.zeros(HISTORY_SIZE),
}

say = rospy.ServiceProxy('/qbo_talk/festival_say', Text2Speach)

def callback(pos_and_dist):
    # Update detections
    STATE['cursor'] = (STATE['cursor'] + 1) % HISTORY_SIZE
    if pos_and_dist.face_detected:
        STATE['detections'][STATE['cursor']] = 1
    else:
        STATE['detections'][STATE['cursor']] = 0

    if not STATE['found_face'] and np.mean(STATE['detections']) > 0.95:
        new_found = True
    elif STATE['found_face'] and np.mean(STATE['detections']) < 0.9:
        new_found = False
    else:
        new_found = STATE['found_face']

    # Utter something appropriate to state change
    if not STATE['found_face'] and new_found:
        say('Hello')
    elif STATE['found_face'] and not new_found:
        say('Goodbye')

    # Update state
    STATE['found_face'] = new_found

def main():
    rospy.init_node('face_looker')
    listener = rospy.Subscriber('/qbo_face_tracking/face_pos_and_dist', FacePosAndDist, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# vim:sw=4:sts=4:et
