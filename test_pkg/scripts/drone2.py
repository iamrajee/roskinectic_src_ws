#!/usr/bin/env python

import rospy
from std_msgs.msg import String
def callback1(data):
    rospy.loginfo("Received loc_drone1 = "+data.data)
def callback2(data):
    rospy.loginfo("Received control_drone1 = "+data.data)


def drone():
    rospy.init_node("drone2", anonymous = True)
    loc_pub = rospy.Publisher("loc_drone2", String, queue_size=10)
    control_pub = rospy.Publisher("control_drone2", String, queue_size=10)
    rospy.Subscriber("loc_drone1", String, callback1)
    rospy.Subscriber("control_drone1", String, callback2)
    
    while not rospy.is_shutdown():
        x,y = 22,22
        roll,pitch,yaw = 1502,1502,1502
        loc_pub.publish(str(x)+","+str(y))
        control_pub.publish(str(roll)+","+str(pitch)+","+str(yaw))
        rospy.Rate(10).sleep()

if __name__ == "__main__":
    try:
            drone()
    except rospy.ROSInterruptException:
            pass
    