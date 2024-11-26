
# Seesaw Balancing simulation
In this project, we mathematically formulated the seesaw dynamics in the presence of disturbances. The balancing point is achieved by controlling the speed of the BLDC motor attached on both ends of the seesaw. The net forces balance the seesaw by using control mechanism such as PID and MPC. The mathematical simulation is performed on MATLAB and the code is available in this repository. 
This is a demo of how the project works:

![til](https://github.com/OsamaAhmad369/SeeSawcontroller/blob/main/demo.gif)

We have also provided simulation on Gazebo using rospy. In Gazebo, seesaw is experiencing disturbance (fixed mass and wind) and IMU sensor with gaussian noise. 
(Note: IMU sensor is attached to one end of seesaw to measure current state of the system).

## 1. PID controller

Open terminal and load the seesaw into gazebo simulator
```
roslaunch seesaw_bldc seesaw_gazebo.launch
```
Open another terminal and run the following command
```
rosrun seesaw_bldc control.py
```
To access each file in new terminal, you may have to run this command
```
source devel/setup.bash
```

