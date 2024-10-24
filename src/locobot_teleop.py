#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

class LocobotTeleop():
    def __init__(self, topic="/locobot/mobile_base/commands/velocity"):
        self.topic = topic

        self.pub = rospy.Publisher(self.topic, Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def turn_right_90(self):
        twist = Twist()
        twist.angular.z = -0.8

        turn_duration = 11

        rospy.loginfo("Turning right 90 degrees")
        start_time = time.time()
        while time.time() - start_time < turn_duration:
            self.pub.publish(twist)
            self.rate.sleep()

        self.stop()

    def move_forward(self, duration=2.0):
        twist = Twist()
        twist.linear.x = 0.75

        rospy.loginfo("Moving forward")
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(twist)
            self.rate.sleep()

        self.stop()

    def turn_left_90(self):
        twist = Twist()
        twist.angular.z = 0.8

        turn_duration = 11

        rospy.loginfo("Turning left 90 degrees")
        start_time = time.time()
        while time.time() - start_time < turn_duration:
            self.pub.publish(twist)
            self.rate.sleep()

        self.stop()

    def stop(self):
        twist = Twist()
        rospy.loginfo("Stopping the robot")
        self.pub.publish(twist)
        self.rate.sleep()

    def move_to_right(self):
        self.turn_right_90()
        self.move_forward(4.5)
        self.turn_left_90()

    def move_to_left(self):
        self.turn_left_90()
        self.move_forward(4.5)
        self.turn_right_90()

if __name__ == '__main__':
    try:
        teleop = LocobotTeleop()
    except rospy.ROSInterruptException:
        pass
