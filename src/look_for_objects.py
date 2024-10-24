#!/usr/bin/env python
import rospy
from locobot_simulation.msg import LogicalImage  
from std_msgs.msg import Float64
import sys
import moveit_commander
import moveit_msgs.msg

rospy.init_node('look_around_node', anonymous=True)

pan_publisher = rospy.Publisher('/locobot/pan_controller/command', Float64, queue_size=10)
tilt_publisher = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=10)

pan_cmd = 0.0
tilt_cmd = 0.3 
delta = 0.01
rate = rospy.Rate(100)
object_detected = True

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander(robot_description = 'locobot/robot_description')
scene = moveit_commander.PlanningSceneInterface('locobot')    
group = moveit_commander.MoveGroupCommander(robot_description = 'locobot/robot_description',ns='locobot',name="interbotix_arm")
gripper_group = moveit_commander.MoveGroupCommander(robot_description = 'locobot/robot_description',ns='locobot',name="interbotix_gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

def open_gripper():
    try:
        gripper_group.set_named_target('Open')
        
        success = gripper_group.go(wait=True)

        if success:
            rospy.loginfo(f"SUCCESSFULLY moved to OPEN state.")
        else:
            rospy.logwarn(f"FAILED to move to OPEN state.")

        rospy.sleep(3)
    except moveit_commander.MoveItCommanderException as e:
        rospy.logerr(f"FAILED to set named target: {e}")


def close_gripper():
    try:
        gripper_group.set_named_target('Closed')
        
        success = gripper_group.go(wait=True)

        if success:
            rospy.loginfo(f"SUCCESSFULLY moved to CLOSED state.")
        else:
            rospy.logwarn(f"FAILED to move to CLOSED state.")

        rospy.sleep(3)

    except moveit_commander.MoveItCommanderException as e:
        rospy.logerr(f"FAILED to set named target: {e}")


def pickup(model):

    rospy.loginfo("INITIATING PICKUP")

    open_gripper()

    model.pose.position.z = model.max.z * 0.85
    group.set_pose_target(model.pose)
    success = group.go(wait=True)

    if success:
        close_gripper()

        try:
            group.set_named_target('Home')

            h_success = group.go(wait=True)

            if h_success:
                rospy.loginfo("OBJECT GRASPED")
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(f"FAILED to set named target: {e}")

    else:
        rospy.logwarn("GRASPING FAILED")


def grasp_object(model):
    global object_detected

    open_gripper()
    close_gripper()

    # model.pose.position.z = model.max.z + (model.max.z * 0.5)
    # rospy.loginfo(f"Grasping object at pose: {model.pose}")
    # group.set_pose_target(model.pose)


    # plan = group.plan()
    # success = group.go(wait=True)
    # if success:
    #     pickup(model=model)
    #     object_detected = True
    # else:

    #     object_detected = True


def publish_topics():
    global pan_cmd, tilt_cmd, delta, object_detected
    while not rospy.is_shutdown():
        if object_detected:
            rospy.loginfo('Publishing command: pan_cmd={}, tilt_cmd={}'.format(pan_cmd, tilt_cmd))
            pan_publisher.publish(pan_cmd)
            tilt_publisher.publish(tilt_cmd)
            if pan_cmd >= 1.8 or pan_cmd <= -1.8:
                delta *= -1
            pan_cmd += delta
            tilt_cmd += 0.01
            rate.sleep()

def start_look_around():
    rospy.loginfo("Starting to look around...")
    publish_topics()              

def camera_callback(data):
    global tilt_cmd, object_detected
    rospy.loginfo("Received camera data:")
    
    for model in data.models:
        rospy.loginfo(f"Model: {model.type}")

        if model.type in ["blue_cube"]:#, "red_cylinder", "red_small_ball", "red_cube", "blue_cylinder", "blue_small_ball"]:
            rospy.loginfo(f"GRASPABLE OBJECT DETECTED: {model.type}")
            object_detected = False
            grasp_object(model)
            tilt_cmd = 0
            break

def camera_listener():
    rospy.Subscriber("/gazebo/locobot/camera/logical_camera_image", LogicalImage, camera_callback)
    start_look_around()

if __name__ == '__main__':
    try:
        camera_listener()
    except rospy.ROSInterruptException:
        pass

moveit_commander.roscpp_shutdown()