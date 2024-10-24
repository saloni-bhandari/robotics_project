#!/usr/bin/env python
import rospy
from locobot_simulation.msg import LogicalImage  
from std_msgs.msg import Float64
from move_robot_arm import grasp_object, shutdown_moveit, move_to_home
import sys
from locobot_teleop import LocobotTeleop

rospy.init_node('look_around_node', anonymous=True)

pan_publisher = rospy.Publisher('/locobot/pan_controller/command', Float64, queue_size=10)
tilt_publisher = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=10)

pan_cmd = 0.0
tilt_cmd = 0.3 
delta = 0.01
rate = rospy.Rate(100)
object_detected = False
blue_target_area = None
teleop = LocobotTeleop()

def publish_topics():
    global pan_cmd, tilt_cmd, delta, object_detected
    while not rospy.is_shutdown():
        if not object_detected:
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
    global tilt_cmd, object_detected, blue_target_area
    rospy.loginfo("Received camera data:")
    
    for model in data.models:
        rospy.loginfo(f"Model: {model.type}")

        # if model.type == "blue_target_area":
        #     rospy.loginfo("BLUE TARGET AREA DETECTED AND STORED.")
        #     blue_target_area = model
        #     move_to_target(blue_target_area)

        if model.type in ["red_cube"]:
            rospy.loginfo(f"GRASPABLE OBJECT DETECTED: {model.type}")
            object_detected = True
            #grasp_object(model)
            #move_to_home()
            teleop.move_to_right()
            teleop.move_to_left()
            rospy.signal_shutdown("moved")
            object_detected = False
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

shutdown_moveit()
