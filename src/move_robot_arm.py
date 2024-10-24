#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import sys

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander(robot_description='locobot/robot_description')
scene = moveit_commander.PlanningSceneInterface('locobot')
group = moveit_commander.MoveGroupCommander(robot_description='locobot/robot_description', ns='locobot', name="interbotix_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

def grasp_object(model):
    model.pose.position.z = model.max.z + 0.1
    rospy.loginfo(f'Model max z-coord: {model.pose.position.z}')
    rospy.loginfo(f"Grasping object at: {model.pose}")
    group.set_pose_target(model.pose)
    plan = group.plan()
    if group.go(wait=True):
        rospy.loginfo("OBJECT GRASPED!")
    else:
        rospy.loginfo("GRASPING FAILED")

def move_to_home():
    group.set_named_target('Home')
    success = group.go(wait=True)
    if(success):
        rospy.loginfo("MOVED TO HOME")
    else:
        rospy.loginfo("COULD NOT MOVE TO HOME")

    joint_vars = group.get_end_effector_link()
    rospy.loginfo(f"JOINT VARS {joint_vars}")

def move_to_target(model):
    model.pose.position.z = model.max.z + 10
    # model.pose.position.x = model.min.x
    # model.pose.position.y = model.min.y
    rospy.loginfo(f'Model max z-coord: {model.pose.position.z}')
    rospy.loginfo(f"MOVING TO: {model.pose}")
    group.set_pose_target(model.pose)
    plan = group.plan()
    if group.go(wait=True):
        rospy.loginfo("MOVED!")
    else:
        rospy.loginfo("MOVING FAILED")

def shutdown_moveit():
    moveit_commander.roscpp_shutdown()
