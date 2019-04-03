#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback1(data):
    rospy.loginfo("Received loc_drone2 = "+data.data)


def drone():
    rospy.init_node("drone1", anonymous = True)
    loc_pub = rospy.Publisher("loc_drone1", String, queue_size=10)
    control_pub = rospy.Publisher("control_drone1", String, queue_size=10)
    loc_drone2 = rospy.Subscriber("loc_drone2", String, callback1)
    
    
    while rospy.is_shutdown() != 0:
        x,y = 1,1
        roll,pitch,yaw = 1501,1501,1501
        loc_pub.publish(str(x)+","+str(y))
        control_pub.publish(str(roll)+","+str(pitch)+","+str(yaw))
        rospy.Rate(10).sleep()

if __name__ == "__main__":
    try:
            drone()
    except rospy.ROSInterruptException:
            pass
    