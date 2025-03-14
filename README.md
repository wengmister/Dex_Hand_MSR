# Dex_Hand_MSR
Author: Zhengyang Kris Weng

Open source release of dexterous robotic hand, MSR Winter 2025. 

This README is currently under construction - more information to come!

# Overview

This is the open source release of MSR dexterous hand V1. The hand has 15 degrees of freedom, and are cable/pulley controlled with servos with "N configuration" - that is, there are 15 servos driving the 15 joints.   

Each finger features 3 DOF - Metacarpal (MCP) adduction/abduction, MCP flexion/extension, and proximal interphalangeal (PIP) joint flexion/extension. I designed a four bar linkage on the distal end of the phalanx, and with this mechanism   motion on PIP turns into a coupled motion on the distal interphalangeal (DIP) joint as well. This configuration emulates the "coupled" movements between human PIP and DIP joints, and also helps reduce the actuator count.   

The thumb also features 3 DOF - Carpometacarpal(CMC) adduction/abduction, MCP adduction/abduction, and PIP flexion/extension. CMC joint has a miniature linear actuator with a lead screw, this helps the joint to be non-backdrivable and keeps weight light.

# Environment Setup

This project currently runs on `ROS2-JAZZY`. To build locally, run:

    colcon build
    . install/setup.bash

For the shadowing demo task currently on V1, we will need to use Google's MediaPipe framework, which can be installed with

    pip install mediapipe

However, in case that you have system managed site-packages, instead of creating a virtual environment (since ROS2 doesn't really work with venv), you'll need to create a `virtual ROS2 environment` package and source site package installed there. You can read more about it <a href="https://github.com/wengmister/Apex-Putter/blob/main/DEPENDENCIES.md" target="_blank">here</a>.  

# Quickstart

For motion shadowing:

    ros2 launch hand_motion_shadowing shadowing.launch.xml usb:=/dev/ttyUSB0

Change usb port based on your device setting.

# Video Demos

Gesture shadowing:    
![shadow](https://github.com/user-attachments/assets/f33a0adb-ee92-4414-a5de-c9753492a840)    


Grasping:    
![grasp](https://github.com/user-attachments/assets/809da993-90ee-4199-834b-34dcadf37cdb)    



Pinching:   
![pinch](https://github.com/user-attachments/assets/91bb948a-0aeb-4b71-be7d-b1b7ebd2675b)    





