# Omniwheels controller

This project contains a simply velocity controller for the three-wheeled omniwheel robot. It publisher topic that enable the user to control the velocity and the rotation of the robot. It is intended to run on the robot. This positioning of the robot is done with with crazyflie deck from Bitcraze.

The topics are the four topics, three on the state of the robot(position, velocity and acceleration) and one that allows velocity command of the robot.
```
omniwheelxx/pos
omniwheelxx/vel
omniwheelxx/acc
omniwheelxx/vel_cmd
```
The omniwheelxx is set in the group namespace in the launch file: launch_controller.launch
```
<group ns = "omniwheel1">
</group>
```

## Dependencies
- Ros Melodic, http://wiki.ros.org/melodic/Installation/Ubuntu
- In order to communicate with the crazy flie, the crazyflie-lib python library by BitCraze is required. A detailed instruction on how to install is found in https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/.
The github repository is
https://github.com/bitcraze/crazyflie-lib-python.git.

- Python Dynamixel library by the Department of Automatic control Lund, is required. It is worth noting that this libraries is done in python3. If you try to install with python2, you will get error. The github repository is at git@gitlab.control.lth.se:anders_blomdell/dynamixel.git.

## Usage
- Put the package in the src folder of your catkin workspace and build.
- To run the omniwheel controller execute the following launch file:
```
roslaunch omniwheel_controller launch_controller.launch
```


## General observations
- Due to the noise in the sensor data, the pid controller is not exact. A better approach if time permit is to implement a kalman filter for the sensor data, to better improve the pid controller
- Issue with device permission usb crazyflie, solution can be found in
https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/
- A explanation on how to configure ssh with ros can be found in: https://github.com/ut-ims-robotics/tutorials/wiki/Running-ROS-over-multiple-computers

- Make sure that the most main computer is the ros master and and source the
configure_ros_master.bash file. The file is in the utills important.
- The parameters for the gains are in the omniwheels_config.yaml file in the config folder.
- It would be nice to define the group namespace in the launch_controller file based on the ID on the omniwheels_config.yaml file.


## Citation
Consider citing this work if you find the code helpful for your projects.

```
@mastersthesis{my-thesis,
  author       = {{Stevedan Omodolor}},
  language     = {{eng}},
  note         = {{TFRT-6181}},
  title        = {{Formation and orientation-based control of UAVs and Coordination with UGVs}},
  year         = {{2022}},
    school = {Department of Automatic Control, Lund University, Sweden},
    url = {https://lup.lub.lu.se/student-papers},
%   number = {TFRT-6181},
}
```
