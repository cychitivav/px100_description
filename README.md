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
    <img src="https://user-images.githubusercontent.com/30636259/167985968-a38b2639-67d4-43f1-95a2-bb6e3385b5a5.png#gh-light-mode-only" width="600px">
    <img src="https://user-images.githubusercontent.com/30636259/167985973-424f5602-4897-4146-8d0e-67edd4a82a86.png#gh-dark-mode-only" width="600px">
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

#### Parameters Denavit-Hartenberg standard 
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

### URDF 


## Launch file




<!-- los resultados obtenidos, los análisis realizados y las conclusiones. -->
