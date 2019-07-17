#!/usr/bin/env python

import rospy
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation

object_name="my_cube"
# object_name="my_box1"

rospy.init_node("moveit_py")
s = PlanningSceneInterface("panda_link0")
s.is_diff="true"
# s.RobotState.is_diff="true"
# eef_link="panda_leftfinger"
eef_link="panda_link0"

print "\n\n\n=============== Press `Enter` to add Box ==============\n"
raw_input()
cube_size=0.1 #s.addCube(object_name, 0.1, 1, 0, 0.5)# add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame
cube_x=1
cube_y=0
cube_z=0.5
s.addBox(object_name, cube_size,cube_size,cube_size, cube_x, cube_y, cube_z)

print "\n\n\n=============== Press `Enter` to list of object ==============\n"
raw_input()
for name in s.getKnownCollisionObjects():
        print(name)

print "\n\n\n=============== Press `Enter` to attach Box ==============\n"
raw_input()
s.attachBox(object_name, cube_size, cube_size, cube_size,cube_x, cube_y, cube_z, eef_link)

print "\n\n\n=============== Press `Enter` to list of object ==============\n"
raw_input()
for name in s.getKnownCollisionObjects():
        print(name)

print "\n\n\n=============== Press `Enter` to list of attached object ==============\n"
raw_input()
for name in s.getKnownAttachedObjects():
        print(name)

if len(s.getKnownCollisionObjects() + s.getKnownAttachedObjects()) == 0:
        print("No objects in planning scene.")

print "\n\n\n=============== Press `Enter` to pick cube ==============\n"
raw_input()
pp = PickPlaceInterface("panda_arm", "hand")# also takes a third parameter "plan_only" which defaults to False
g = Grasp()# fill in g
print(g)
pp.pickup(object_name, [g, ], support_name = "supporting_surface")
l = PlaceLocation()# fill in l
print(l)
pp.place(object_name, [l, ], goal_is_eef = True, support_name = "supporting_surface")

print "\n\n\n=============== Press `Enter` to De-attach Box ==============\n"
raw_input()
s.removeAttachedObject(object_name)

print "\n\n\n=============== Press `Enter` to remove cube ==============\n"
raw_input()
s.removeCollisionObject(object_name)# remove the cube