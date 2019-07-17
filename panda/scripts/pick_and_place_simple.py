#!/usr/bin/env python

import rospy
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation

object_name="my_cube"

rospy.init_node("moveit_py")
s = PlanningSceneInterface("panda_link0")
s.addCube(object_name, 0.1, 1, 0, 0.5)# add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame

# pp = PickPlaceInterface("panda_arm", "hand")# also takes a third parameter "plan_only" which defaults to False
# g = Grasp()# fill in g
# pp.pickup(object_name, [g, ], support_name = "supporting_surface")
# l = PlaceLocation()# fill in l
# pp.place("object_name" [l, ], goal_is_eef = True, support_name = "supporting_surface")

s.removeCollisionObject(object_name)# remove the cube