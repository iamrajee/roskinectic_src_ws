#!/usr/bin/env python
'''
****************** Shortforms **************
r : robot
s : scene
gn : group_name
gns : group_names
wp : waypoints
dt : display_trajectory
dt_pub : display_trajectory_publisher
********************************************
'''

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False
  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)
  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  return True

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    r = moveit_commander.RobotCommander()
    s = moveit_commander.PlanningSceneInterface()
    gn = "panda_arm"
    g = moveit_commander.MoveGroupCommander(gn)
    dt_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    # ======================== Getting Basic Information ================================= #
    planning_frame = g.get_planning_frame()
    print "----------- Reference frame: %s" % planning_frame
    eef_link = g.get_end_effector_link()
    print "----------- End effector: %s" % eef_link
    gns = r.get_group_names()
    print "----------- Robot Groups:", r.get_group_names()
    print "----------- Printing robot state"
    print r.get_current_state()
    print ""
    # ===================================================================================== #
    # Misc variables
    self.box_name = 'box'
    self.r = r
    self.s = s
    self.g = g
    self.dt_pub = dt_pub
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.gns = gns

  #================================ Planning to a Joint Goal ==================================#
  def go_to_joint_state(self,j1):
    g = self.g

    current_pose = self.g.get_current_pose()
    joint_goal = g.get_current_joint_values()
    joint_goal[0] = j1
    joint_goal[1] = 0
    joint_goal[2] = 0
    # joint_goal[3] = -pi/2
    # joint_goal[4] = 0
    # joint_goal[5] = pi/3
    # joint_goal[6] = 0

    g.go(joint_goal, wait=True)
    g.stop()

    current_joints = self.g.get_current_joint_values()
    print("required=>",self.g.get_current_pose().pose)
    # return all_close(joint_goal, current_joints, 0.01)
    return current_pose

  #================================ Planning to a Pose Goal ==================================#
  def go_to_pose_goal(self,pose_goal):
    g = self.g
    print(self.g.get_current_pose())
    # pose_goal = self.g.get_current_pose().pose
    # pose_goal.position.x -= 0.02
    # pose_goal.position.y -= 0.02
    # pose_goal.position.z -= 0.01

    # pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.header.frame_id = "Base"
    # pose_goal = pose_goal.pose
    # pose_goal.position.x = 0.12
    # pose_goal.position.y = 0.12
    # pose_goal.position.y = -pose_goal.position.y
    # pose_goal.position.z = 0.24
    # pose_goal.orientation.w = 0.92

    # pose_goal = self.g.get_current_pose().pose
    # pose_goal.position.x -= 0.02
    # pose_goal.position.x +=tm
    # pose_goal.position.z -= 0.01

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.121421276909
    # pose_goal.position.y = -0.121412049201
    # pose_goal.position.z = 0.247687219645

    g.set_pose_target(pose_goal)
    plan = g.go(wait=True)
    g.stop()
    g.clear_pose_targets()
    current_pose = self.g.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  #================================ Cartesian Paths/Waypoints ==================================#
  def plan_cartesian_path(self, scale=1):
    g = self.g
    wp = []
    wpose = g.get_current_pose().pose
    wpose.position.z += scale * 0.01  # First move up (z)
    wpose.position.y += scale * 0.01  # and sideways (y)
    wp.append(copy.deepcopy(wpose))
    wpose.position.x += scale * 0.01  # Second move forward/backwards in (x)
    wp.append(copy.deepcopy(wpose))
    wpose.position.y -= scale * 0.01  # Third move sideways (y)
    wp.append(copy.deepcopy(wpose))

    (plan, fraction) = g.compute_cartesian_path(wp, 0.005, 0.0)
    return plan, fraction #just planning

  #================================ Displaying a Trajectory ==================================#
  def dt(self, plan):
    r = self.r
    dt_pub = self.dt_pub
    dt = moveit_msgs.msg.DisplayTrajectory()
    dt.trajectory_start = r.get_current_state()
    dt.trajectory.append(plan)
    dt_pub.publish(dt)

  #================================ Executing a Plan ==================================#
  def execute_plan(self, plan):
    g = self.g
    g.execute(plan, wait=True)

  #================================ Ensuring Collision Updates Are Receieved ==================================#
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    s = self.s
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = s.get_attached_objects([box_name])# Test if the box is in attached objects
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in s.get_known_object_names()# Test if the box is in the s.
      if (box_is_attached == is_attached) and (box_is_known == is_known):# Test if we are in the expected state
        return True
      rospy.sleep(0.1)#give other threads time
      seconds = rospy.get_time()
    return False# If we exited the while loop without returning then we timed out

  #================================ Adding Objects to the Planning Scene ==================================#
  def add_box(self, timeout=4):
    box_name = self.box_name
    s = self.s
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "Link9" #leftfinger
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x += 0.08
    # box_pose.pose.position.y = 0.12
    box_pose.pose.position.z -= 0.07
    box_name = "box"
    s.add_box(box_name, box_pose, size=(0.025, 0.025, 0.025))
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  #================================ Attaching Objects to the Robot ==================================#
  def attach_box(self, timeout=4):
    box_name = self.box_name
    r = self.r
    s = self.s
    eef_link = self.eef_link
    gns = self.gns
    # grasping_group = 'hand'
    grasping_group = 'panda_arm'
    touch_links = r.get_link_names(group=grasping_group)
    s.attach_box(eef_link, box_name, touch_links=touch_links)
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  #================================ Detaching Objects to the Robot ==================================#
  def detach_box(self, timeout=4):
    box_name = self.box_name
    s = self.s
    eef_link = self.eef_link
    s.remove_attached_object(eef_link, name=box_name)
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  #================================ Removing Objects from the Planning Scene ==================================#
  def remove_box(self, timeout=4):
    box_name = self.box_name
    s = self.s
    s.remove_world_object(box_name)
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

#************************************ MAIN ************************************#
def main():
    try:
        print "----------- Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
        # raw_input()
        tutorial = MoveGroupPythonIntefaceTutorial()

        # while(True):
        #   print "-----------\n"
        #   ta = raw_input()
        #   tutorial.go_to_pose_goal(tutorial.go_to_joint_state(pi*float(ta)))

        print "----------- Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_joint_state(pi/4)
        # while(True):
        #   print "-----------\n"
        #   ta = raw_input()
        #   tutorial.go_to_joint_state(pi*float(ta))
        # print "----------- Press `Enter` to execute a movement using a pose goal ..."
        # raw_input()
        # tutorial.go_to_pose_goal()
        # while(True):
        #   print "-----------\n"
        #   ta = raw_input()
        #   tutorial.go_to_pose_goal(float(ta))
        # print "----------- Press `Enter` to plan and display a Cartesian path ..."
        # raw_input()
        # cartesian_plan, fraction = tutorial.plan_cartesian_path()
        # print "----------- Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # raw_input()
        # tutorial.dt(cartesian_plan)
        # print "----------- Press `Enter` to execute a saved path ..."
        # raw_input()
        # tutorial.execute_plan(cartesian_plan)
        print "----------- Press `Enter` to add a box to the planning scene ..."
        raw_input()
        tutorial.add_box()
        print "----------- Press `Enter` to attach a Box to the uarm robot ..."
        raw_input()
        tutorial.attach_box()
        print "----------- Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_joint_state(-pi/4)
        # print "----------- Press `Enter` to plan and execute a path with an attached collision object ..."
        # raw_input()
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)
        print "----------- Press `Enter` to detach the box from the uarm robot ..."
        raw_input()
        tutorial.detach_box()
        print "----------- Press `Enter` to remove the box from the planning scene ..."
        raw_input()
        tutorial.remove_box()
        print "----------- Python tutorial demo complete!"
    except rospy.ROSInterruptException:
      return
    except KeyboardInterrupt:
      return

#************************************ RUN ************************************#
if __name__ == '__main__':
    main()