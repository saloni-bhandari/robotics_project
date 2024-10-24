# Robotics Final Project: Robot Arm Control Pipeline

## Overview

This project is a control pipeline for a Locobot, which works in the following steps:
1. **Look around** the environment and detect objects using its camera.
2. **Grasp objects** with its robotic arm.
3. **Move the object** to a specified target area.
4. **Return to the starting position** and resume scanning for new objects.

## File Structure

We worked across three main Python scripts:

1. **`detect_objects.py`**:
   - This script handles the camera feed and object detection. The robot looks around by panning and tilting its camera until it detects a predefined object (e.g., a red cube). Once an object is detected, it triggers the grasping and movement functions. After grasping and placing the object, it continues to look around for a new object. It also handles the bulk of the code and interacts with the functions from the other scripts so at launch 
   
2. **`move_robot_arm.py`**:
   - This script controls the robot arm's movement using **MoveIt**. It contains functions to open and close the gripper, lower the arm to grasp an object, and move the arm to a home position or target location.
   
3. **`locobot_teleop.py`**:
   - This script provides moves the robot by publishing to the **/locobot/mobile_base/commands/velocity** topic. It allows the robot to turn 90 degrees to the left or right, or forward and is used to move the robot to the target areas.

## Running the Code

To run the project, follow these steps:

### 1. Set up the Environment

Use the following command to run the simulation first.
```
roslaunch locobot_moveit xslocobot_moveit.launch
```
Once Gazebo is running, make sure to "start" the simulation by pressing the play button so move it can be used for planning and executing.
### 2. Run the detection code

in another terminal run the following command to run the python code:
```
python detect_objects.py
```