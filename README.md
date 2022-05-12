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
    <img src="https://user-images.githubusercontent.com/30636259/167949370-5cff0110-3d98-4cd4-ac8a-29e5eedce16b.png#gh-light-mode-only" width="600px">
    <img src="https://user-images.githubusercontent.com/30636259/167949384-ac9090ae-45ae-44c3-ae02-bd3d5578fe89.png#gh-dark-mode-only" width="600px">
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

## Python controller


<!-- los resultados obtenidos, los análisis realizados y las conclusiones. -->
