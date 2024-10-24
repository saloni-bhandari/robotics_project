#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import sys

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander(robot_description='locobot/robot_description')
scene = moveit_commander.PlanningSceneInterface('locobot')
group = moveit_commander.MoveGroupCommander(robot_description='locobot/robot_description', ns='locobot', name="interbotix_arm")
gripper_group = moveit_commander.MoveGroupCommander(robot_description='locobot/robot_description', ns='locobot', name="interbotix_gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


def open_gripper():
    try:
        # use predefined values from named target 'Open'
        gripper_group.set_named_target('Open')
        
        success = gripper_group.go(wait=True)

        if success:
            rospy.loginfo(f"SUCCESSFULLY moved to OPEN state.")
        else:
            rospy.logwarn(f"FAILED to move to OPEN state.")

    except moveit_commander.MoveItCommanderException as e:
        rospy.logerr(f"FAILED to set named target: {e}")


def close_gripper():
    try:
        # use predefined values from named target 'Closed'
        gripper_group.set_named_target('Closed')
        
        success = gripper_group.go(wait=True)

        if success:
            rospy.loginfo(f"SUCCESSFULLY moved to CLOSED state.")
        else:
            rospy.logwarn(f"FAILED to move to CLOSED state.")

    except moveit_commander.MoveItCommanderException as e:
        rospy.logerr(f"FAILED to set named target: {e}")


def lower_gripper():
    current_pose = group.get_current_pose().pose

    # lower the gripper height
    current_pose.position.z -= 0.3
    rospy.loginfo(f"Lowering gripper by {0.3} meters.")

    group.set_pose_target(current_pose)

    success = group.go(wait=True)

    if success:
        rospy.loginfo("Gripper successfully lowered.")
    else:
        rospy.logwarn("Failed to lower the gripper.")


def grasp_object(model):

    # Open the gripper, move arm towards object, close gripper
    
    open_gripper()
    model.pose.position.z = model.max.z * 0.9 # update model pose to 90% of object's height
    # model.pose.position.x = model.max.x
    # model.pose.position.y = model.min.y
    rospy.loginfo(f"Grasping object at: {model.pose}")
    group.set_pose_target(model.pose)
    plan = group.plan()
    if group.go(wait=True):
        close_gripper()     # if it moved to position, grasp object
        rospy.loginfo("OBJECT GRASPED!")
    else:
        rospy.loginfo("GRASPING FAILED")

def move_to_home():
    # Move arm to predefined 'Home' position
    
    group.set_named_target('Home')
    success = group.go(wait=True)
    if(success):
        rospy.loginfo("MOVED TO HOME")
    else:
        rospy.loginfo("COULD NOT MOVE TO HOME")
    
    return success


def shutdown_moveit():
    moveit_commander.roscpp_shutdown()
