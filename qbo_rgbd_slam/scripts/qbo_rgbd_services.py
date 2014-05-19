#!/usr/bin/env python  

import roslib
import rospy
import tf
import Tkinter as tk
from std_srvs.srv import *
 
class ServiceWidget(tk.Frame):
    def __init__(self, master):
        # Initialize window using the parent's constructor
        tk.Frame.__init__(self,
                          master,
                          width=300,
                          height=200)
        # Set the title
        self.master.title('RGBD Service Widget')
 
        # This allows the size specification to take effect
        self.pack_propagate(0)
 
        # We'll use the flexible pack layout manager
        self.pack()
 
        # The recipient text entry control and its StringVar
        self.recipient_var = tk.StringVar()
        self.recipient = tk.Entry(self,
                                  textvariable=self.recipient_var)
        self.recipient_var.set('world')
 
        # The go button
        self.save_button = tk.Button(self,
                                   text='Save Map',
                                   command=self.saveSrvRequest)

        self.load_button = tk.Button(self,
                                   text='Load Map',
                                   command=self.loadSrvRequest)

        self.generate_button = tk.Button(self,
                                   text='Generate 2D Map',
                                   command=self.generateMapSrvRequest)

        self.publish_button = tk.Button(self,
                                   text='Publish PCD Map',
                                   command=self.pubPcdSrvRequest)

        self.toggle_mapping_button = tk.Button(self,
                                   text='Toggle Mapping',
                                   command=self.toggleMappingSrvRequest)
 
        # Put the controls on the form
        self.save_button.pack(fill=tk.X, side=tk.BOTTOM)
        self.load_button.pack(fill=tk.X, side=tk.BOTTOM)
        self.generate_button.pack(fill=tk.X, side=tk.BOTTOM)
        self.publish_button.pack(fill=tk.X, side=tk.BOTTOM)
        self.toggle_mapping_button.pack(fill=tk.X, side=tk.BOTTOM)

    def run(self):
        ''' Run the app '''
        self.mainloop()
    def saveSrvRequest(self):
      rospy.wait_for_service('save_keyframes')
      try:
        save_kf = rospy.ServiceProxy('save_keyframes', Empty)
        resp = save_kf()
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    def loadSrvRequest(self):
      rospy.wait_for_service('load_keyframes')
      try:
        load_kf = rospy.ServiceProxy('load_keyframes', Empty)
        resp = load_kf()
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    def generateMapSrvRequest(self):
      rospy.wait_for_service('generate_2d_map')
      try:
        gen_kf = rospy.ServiceProxy('generate_2d_map', Empty)
        resp = gen_kf()
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    def pubPcdSrvRequest(self):
      rospy.wait_for_service('publish_keyframes')
      try:
        pub_kf = rospy.ServiceProxy('publish_keyframes', Empty)
        resp = pub_kf()
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    def toggleMappingSrvRequest(self):
      rospy.wait_for_service('stop_mapping')
      try:
        stop_kf = rospy.ServiceProxy('stop_mapping', Empty)
        resp = stop_kf()
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    app = ServiceWidget(tk.Tk())
    app.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass