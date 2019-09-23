#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

class GazeboConnection():
    
    def __init__(self):
        
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    
    def pauseSim(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException, e:
            print ("/gazebo/pause_physics service call failed")
        
    def unpauseSim(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException, e:
            print ("/gazebo/unpause_physics service call failed")
        
    def resetSim(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_simulation service call failed")