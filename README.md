# PincherX 100 robot arm package
This repo contains the second practice lab for robotics class at Universidad Nacional de Colombia

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

<!-- los resultados obtenidos, los análisis realizados y las conclusiones. -->
