# PincherX 100 robot arm package
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
catkin create pkg hello_turtle -m "Cristian Chitiva" "cychitivav@unal.edu.co" -m "Brayan Estupinan" "blestupinanp@unal.edu.co" -l "MIT" --catkin-deps rospy dynamixel_workbench_controllers 
```

The dependencies of this package are:
* rospy
* dynamixel_workbench_controllers 

## Robot description
To properly move the robot, a model was first created taking into account the measurements made with a vernier caliper.
<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/168069841-0ad7f173-1fb5-4adf-90f7-7241f0089d0b.png#gh-light-mode-only" width="600px">
    <img src="https://user-images.githubusercontent.com/30636259/168069834-93b07fa0-41b0-432c-b5ac-eff768c2b66e.png#gh-dark-mode-only" width="600px">
</p>

where the measurements are:
<ul>
    <li> 
        <img src="https://render.githubusercontent.com/render/math?math=L_1=44.5\ \ mm#gh-light-mode-only"> <img src="https://render.githubusercontent.com/render/math?math=\color{white}L_1=44.5\ mm#gh-dark-mode-only"> 
    </li>
    <li> 
        <img src="https://render.githubusercontent.com/render/math?math=L_2=101\ \ mm#gh-light-mode-only"> <img src="https://render.githubusercontent.com/render/math?math=\color{white}L_2=101\ mm#gh-dark-mode-only">
    </li>
    <li> 
        <img src="https://render.githubusercontent.com/render/math?math=L_3=101\ \ mm#gh-light-mode-only"> <img src="https://render.githubusercontent.com/render/math?math=\color{white}L_3=101\ mm#gh-dark-mode-only"> 
    </li>
    <li>   
        <img src="https://render.githubusercontent.com/render/math?math=L_4=109\ \ mm#gh-light-mode-only"> <img src="https://render.githubusercontent.com/render/math?math=\color{white}L_4=109\ mm#gh-dark-mode-only"> 
    </li>
    <li>
        <img src="https://render.githubusercontent.com/render/math?math=L_m=31.5\ \ mm#gh-light-mode-only"> <img src="https://render.githubusercontent.com/render/math?math=\color{white}L_m=31.5\ mm#gh-dark-mode-only"> 
    </li>
</ul>

#### Denavit-Hartenberg standard parameters
<table align="center">
    <tr align="center">
        <th>
            <img src="https://render.githubusercontent.com/render/math?math=\mathbf{i}#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}\mathbf{i}#gh-dark-mode-only"> 
        </th>
        <th>
            <img src="https://render.githubusercontent.com/render/math?math=\mathbf{\theta_i}#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}\mathbf{\theta_i}#gh-dark-mode-only"> 
        </th>
        <th>
            <img src="https://render.githubusercontent.com/render/math?math=\mathbf{d_i}#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}\mathbf{d_i}#gh-dark-mode-only"> 
        </th>
        <th>
            <img src="https://render.githubusercontent.com/render/math?math=\mathbf{a_i}#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}\mathbf{a_i}#gh-dark-mode-only"> 
        </th>
        <th>
            <img src="https://render.githubusercontent.com/render/math?math=\mathbf{\alpha_i}#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}\mathbf{\alpha_i}#gh-dark-mode-only"> 
        </th>
    </tr>
    <tr align="center">
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=1#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}1#gh-dark-mode-only"> 
        </td> 
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=q_1#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}q_1#gh-dark-mode-only"> 
        </td>  
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=L_1#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}L_1#gh-dark-mode-only"> 
        </td>
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=0#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}0#gh-dark-mode-only"> 
        </td>
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=-\dfrac{\pi}{2}#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}-\dfrac{\pi}{2}#gh-dark-mode-only"> 
        </td>
    </tr>
    <tr align="center">
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=2#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}2#gh-dark-mode-only"> 
        </td> 
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=q_2#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}q_2#gh-dark-mode-only"> 
        </td>  
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=0#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}0#gh-dark-mode-only"> 
        </td>
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=\sqrt{L_2^2%2bL_m^2}#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}\sqrt{L_2^2%2bL_m^2}#gh-dark-mode-only"> 
        </td>
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=0#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}0#gh-dark-mode-only"> 
        </td>
    </tr>
    <tr align="center">
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=3#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}3#gh-dark-mode-only"> 
        </td> 
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=q_3#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}q_3#gh-dark-mode-only"> 
        </td>  
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=0#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}0#gh-dark-mode-only"> 
        </td>
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=L_3#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}L_3#gh-dark-mode-only"> 
        </td>
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=0#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}0#gh-dark-mode-only"> 
        </td>
    </tr>
    <tr align="center">
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=4#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}4#gh-dark-mode-only"> 
        </td> 
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=q_4#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}q_4#gh-dark-mode-only"> 
        </td>  
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=0#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}0#gh-dark-mode-only"> 
        </td>
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=L_4#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}L_4#gh-dark-mode-only"> 
        </td>
        <td>
            <img src="https://render.githubusercontent.com/render/math?math=0#gh-light-mode-only"> 
            <img src="https://render.githubusercontent.com/render/math?math=\color{white}0#gh-dark-mode-only"> 
        </td>
    </tr>
</table>

### URDF (Unified Robot Description Format)
In order to visualize the robot in RViz, we created a [URDF file](urdf/robot.urdf). This file contains the information on how the links and joints of the robot are connected, in addition to the [CAD models](https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/meshes)[^pxstore] for a closer view of reality. The structure to create a urdf can be seen in the following image:

<p align="center">
    <img alt="joints and links urdf" src"http://wiki.ros.org/urdf/XML/joint?action=AttachFile&do=get&target=joint.png">
</p>


## Launch file
To display the joints and links correctly in RViz, it is necessary to run the nodes that publish the robot and joint states. These nodes must be run every time you want to check the package and it becomes tedious, so you create a [launch file](launch/px100_rviz.launch) that runs the simulation.


<!-- los resultados obtenidos, los análisis realizados y las conclusiones. -->


## References
[^pxstore] PincherX 100 Robot Arm - X-Series Robotic Arm..