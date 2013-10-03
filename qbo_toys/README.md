# Some toy nodes for the Q.bo

These are some silly little nodes which demonstrate how to talk to Q.bo via
rospy.

## head\_wander.py

An example of how to publish information to a topic and call a service. In this
case, publish random head positions to make the Q.bo look around and make
comments on the world. Requires the Q.bo be launched with
``speech_synthesis:=true``.

## hello\_goodbye.py

Uses the face tracking nodes to follow faces. If a face is found, Q.bo says
"hello". If a face is lost, Q.bo says "goodbye". An example of subscribing to a
topic and calling a service.

