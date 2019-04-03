#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def callback1(data):
    rospy.loginfo("Received loc_drone2 = "+data.data)
def callback2(data):
    rospy.loginfo("Received control_drone2 = "+data.data)
def callback3(msg):
    x = msg.pose.pose.position.x
    rospy.loginfo("******"+str(x))

def drone():
    rospy.init_node("drone1", anonymous = True)
    loc_pub = rospy.Publisher("loc_drone1", String, queue_size=10)
    control_pub = rospy.Publisher("control_drone1", String, queue_size=10)
    rospy.Subscriber("loc_drone2", String, callback1)
    rospy.Subscriber("control_drone2", String, callback2)
    rospy.Subscriber("/odom", Odometry, callback3)
    
    
    while not rospy.is_shutdown():
        x,y = 11,11
        roll,pitch,yaw = 1501,1501,1501
        loc_pub.publish(str(x)+","+str(y))
        control_pub.publish(str(roll)+","+str(pitch)+","+str(yaw))
        rospy.Rate(10).sleep()

if __name__ == "__main__":
    try:
            drone()
    except rospy.ROSInterruptException:
            pass
    