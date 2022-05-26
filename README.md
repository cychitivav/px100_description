# PincherX 100 description
This repo contains the second practice lab for robotics class at Universidad Nacional de Colombia

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/167962176-b8172b07-c769-4a7b-8420-db518c59fffa.png" width="300px"/>
</p>


### Requirements
* ROS Noetic
* Dynamixel workbench controllers package (`sudo apt install ros-noetic-dynamixel-workbench`)
* ROS toolbox for MATLAB
* Pynput library for python (`pip install pynput` or `pip3 install pynput`)
* [PincherX 100 Robot arm](https://www.trossenrobotics.com/pincherx-100-robot-arm.aspx)

## Package creation 
In order to create a new package, we used the next command (in the workspace folder):

```bash
catkin create pkg px100_description -m "Cristian Chitiva" "cychitivav@unal.edu.co" -m "Brayan Estupinan" "blestupinanp@unal.edu.co" -l "MIT" --catkin-deps rospy dynamixel_workbench_controllers 
```

The dependencies of this package are:
* rospy
* dynamixel_workbench_controllers 

## Robot description
To properly move the robot, a model was first created taking into account the measurements made with a vernier caliper.
<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/169600958-763d1628-b7bb-475c-a5cc-56120d1d3ff9.svg#gh-light-mode-only" width="600px">
    <img src="https://user-images.githubusercontent.com/30636259/169601364-d561fece-3c1c-4a8e-9616-bae1d36fa925.svg#gh-dark-mode-only" width="600px">
</p>

where the measurements are:
* $L_1=44.5~mm$
* $L_2=101~mm$
* $L_3=101~mm$
* $L_4=109~mm$
* $L_m=31.5~mm$

### Denavit-Hartenberg standard parameters
<div align="center">

| $\mathbf{i}$ | $\mathbf{\theta_i}$ | $\mathbf{d_i}$ | $\mathbf{a_i}$ | $\mathbf{\alpha_i}$ |
|:------------:|:-------------------:|:--------------:|:--------------:|:-------------------:|
|      $1$     |         $q_1$       |      $L_1$     |       $0$      |   $-\frac{\pi}{2}$  |
|      $2$     |         $q_2$       |       $0$      |      $L_r$     |         $0$         |
|      $3$     |         $q_3$       |       $0$      |      $L_3$     |         $0$         |
|      $4$     |         $q_4$       |       $0$      |      $L_4$     |         $0$         |

</div>

> __Note__: $L_r=\sqrt{L_m^2+L_2^2}$

## ROS
Based on the _dynamixel workbench packages_ in ROS, we created a Python script that calls the __dynamixel_command service__ to move each of the manipulator's joints (waist, shoulder, elbow, wrist). This movement is done between two characteristic positions (__home__ and __goal__) and can be switched using the following keys:

* <kbd>w</kbd>: Select next joint (if are selected the waist, pass to the shoulder)
* <kbd>s</kbd>: Select previous joint (if are selected the gripper, pass to the wrist)
* <kbd>a</kbd>: Go the selected joint to home position.
* <kbd>d</kbd>: Go the selected joint to goal position.

> __Note__: The change between joints is cyclic, so if you select the gripper, the next joint will be the waist and vice versa.

The code used to move the manipulator uses the _pynput_ library to detect the keys pressed, and the *dynamixel_command service* to send commands to dynamixel motors. The code is available in this [file](scripts/key_control.py).

### URDF (Unified Robot Description Format)
In order to visualize the robot in RViz, we created a [URDF file](urdf/robot.urdf). This file contains the information on how the links and joints of the robot are connected, in addition to the [CAD models](https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/meshes)[^pxstore] for a closer view of reality. The structure to create a urdf can be seen in the following image[^urdf]:

<p align="center">
    <a href="http://wiki.ros.org/urdf/Tutorials">
        <img alt="joints and links urdf" src="https://user-images.githubusercontent.com/30636259/168392285-429063e7-f742-454e-a321-bfebf15f6370.png" width="300px">
    </a>
</p>

Finally, the robot model in RViz is:

<p align="center">
    <img alt="rviz" src="https://user-images.githubusercontent.com/30636259/168394476-701abd29-d1d5-40da-b99d-63ff6d952e59.png" width="700px">
</p>


#### Launch file
To display the joints and links correctly in RViz, it is necessary to run the nodes that publish the robot and joint states (and RViz node). These nodes must be run every time you want to check the package and it becomes tedious, so you create a launch file that runs the dynamixel packages and Rviz.

This package has two launch files, the [first file](launch/px100_rviz.launch) to run RViz with a joint state publisher gui. And the [second file](launch/px100.launch) runs Rviz but the [python file](scripts/key_control.py) is used to move the joints.

##### Execution
The first launch file can be executed with the following command:

```bash
roslaunch px100_description visualize.launch
```

and the second launch file with the following command:

```bash
roslaunch px100_description px100.launch run_dynamixel:=true
```
> __Note:__ The second launch file can receive an argument to run or not the *dynamixel_workbench* packages, which is responsible for publishing the joint states by making a remap of the **/dynamixel_workbench/joint_states** to **/joint_states** topic.

#### Run python node key control
To use the python node in launch with rviz, you need to run the following command:

```bash
rosnode kill /joint_state_publisher_gui
rosrun px100_description key_control.py True
```

##### Configuration files
The files to setup the parameter needed to run the *dynamixel_workbench* package are in the [configuration folder](config/).


### Bonus  
In the urdf we add the cad model of the gripper fingers, along with its respective prismatic joints and the mimic attribute to establish a relationship between the movement of the last motor (joint 4) and the movement of the gripper.

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/168398263-fbae64a6-0c0a-42f4-afa5-5d3920242cc4.gif" width="400px">
</p>

## Robotics toolbox by Peter Corke
To create the robot in Peter Corke's toolbox for MATLAB, we use the *SerialLink* object and the [DH parameter table](#Denavit-Hartenberg-standard-parameters).

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/168400374-2859f4a7-c929-43c5-bf62-8fb3b9c55550.svg" width="400px">
</p>

In order to display the `noa` frame, we use the *tool* property in the SerialLink object. Other configurations of the robot are shown in the following image:

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/168400566-6522913a-0214-4ad1-b81c-0e90264ff7ed.svg" width="400px">
</p>

1. $\begin{bmatrix}\pi/6&\pi/3&-\pi&3\pi/4\end{bmatrix}$
1. $\begin{bmatrix}\pi/3&\pi/6&\pi/7&3\pi/7\end{bmatrix}$
1. $\begin{bmatrix}-\pi/6&\pi/5&\pi/4&\pi\end{bmatrix}$
1. $\begin{bmatrix}\pi&\pi/9&-\pi/2&-3\pi/4\end{bmatrix}$

> The development of this code is in [px file](matlab/px.m).

## MATLAB connection
To establish the MATLAB connection with the dynamixel package, the tutorial described at this [repo](https://github.com/fegonzalez7/rob_unal_clase4) was used. To send each joint position, we create a ROS client and the position and id of each motor is sent with a message.

Finally, the connection with MATLAB, ROS and python is shown in the following video:

<!-- los resultados obtenidos, los análisis realizados y las conclusiones. -->
https://user-images.githubusercontent.com/30636259/168402959-8d4c3c9a-d791-44a3-9c8f-2eef6cf14a95.mp4

[Youtube](https://www.youtube.com/watch?v=nmZt0jveYos)


## References
[^pxstore]: PincherX 100 Robot Arm - X-Series Robotic Arm..
[^urdf]: [Urdf/Tutorials - ROS Wiki.](http://wiki.ros.org/urdf/Tutorials)