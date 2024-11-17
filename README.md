# Robotics: Dynamics and Control (Monsoon 2024) @ IIITH
Final Project for Robotics: Dynamics and Control(EC4.401), IIITH
- Professor: [Nagamanikandan Govindan](https://www.iiit.ac.in/faculty/nagamanikandan-govindan/)

## Table of contents

- [Robotics: Dynamics and Control (Monsoon 2024) @ IIITH](#robotics-dynamics-and-control-monsoon-2024--iiith)
  - [Table of contents](#table-of-contents)
  - [Problem Statement](#problem-statement)
  - [Setup](#setup)
  - [Common Commands](#common-commands)

## Problem Statement

The aim is to track the trajectory of a flying object using a 6DoF Manipulator with the goal of minimizing the error between the end-effector pose and the object's pose. All IK,FK,Dynamics,etc have to be done manually and not through external packages.

## Setup
- To pull latest docker image
    ```bash
    docker pull ghcr.io/soham2560/humble-garden:latest
    ```
- To start container
    - Open Command Pallete with `Ctrl+Shift+P`
    - Select Option to Rebuild and Reopen Container
    - You can use `Import Libs` button once container has opened to import dependent libs
    - Use `Build WS` button to build workspace

## Common Commands
- Launch
    ```bash
    ros2 launch robot_bringup robot.launch.py use_rviz:=True
    ```

- Sample Command
    ```bash
    ros2 topic pub /ur/arm_controller/commands std_msgs/msg/Float64MultiArray '{data: [1.0, 2.0, 3.0 ,1.0, 2.0, 3.0]}'
    ```

Note: The README's in this repository are inspired by [this](https://github.com/TheProjectsGuy/MR21-CS7.503)