# Stevedan Master's thesis Formation and coordination of UAV and UGV. 
This folder contains the implementation of the formation and coordination control on the real robot. It uses the Crazyflie drone the three-wheeled omniwheel robot.

## Dependencies
- ROS Melodic http://wiki.ros.org/melodic/Installation/Ubuntu
- Crazyswam(original library) https://crazyswarm.readthedocs.io/en/latest/

## Important configuration files
- crazyfliesConfig.yaml: This file contains the configuration for the crazyswarm library.
```
crazyflies:
- id: 1
  channel: 90
  initialPosition: [0.0, 0.0, 0.0]
  type: default
- id: 2
  channel: 90
  initialPosition: [0.0, 0.0, 0.0]
  type: default
- id: 3
  channel: 80
  initialPosition: [0.0, 0.0, 0.0]
  type: default
# - id: 4
#   channel: 80
#   initialPosition: [0.0, 0.0, 0.0]
#   type: default
```
- formationConfig.yaml: The formation gain for both the UAVs and UGVs are in this file.

- missionConfig.yaml: This is an important file that contains the definition of the formation mission.
```
n_drones: 3
n_ugvs: 2
hz_freq: 50 # controller frquency
include_obstacle: false
include_orientation: true
n_obs: 0
log_name: "/home/spot/steven_master_ws/src/coordination_formation_control_pkg/results/experiment_6_formation_3_uav_2_ugv/result"
enable_log: true
```
- mission_controller.launch: Based on the number of drone, modify the the launch file accordingly. For example, for a 2 UGV and 3 UAV formation, the launch file should look like this.
```
<group ns="omniwheel_1">
   <node name="omniwheel_node" pkg="coordination_formation_control_pkg" type="omniwheel_node" output="screen" args="1" launch-prefix="xterm -e">
   </node>
</group>
<group ns="omniwheel_2">
    <node name="omniwheel_node" pkg="coordination_formation_control_pkg" type="omniwheel_node" output="screen" args="2" launch-prefix="xterm -e">
    </node>
</group>
<!-- Initialize all the drones first -->
<group ns="uav_1">
<node name="drone_node" pkg="coordination_formation_control_pkg" type="drone_node" output="screen" args="1" launch-prefix="xterm -e">
</node>
</group>

<group ns="uav_2">
<node name="drone_node" pkg="coordination_formation_control_pkg" type="drone_node" output="screen" args="2" launch-prefix="xterm -e">
</node>
</group>
<group ns="uav_3">
 <node name="drone_node" pkg="coordination_formation_control_pkg" type="drone_node" output="screen" args="3" launch-prefix="xterm -e">
 </node>
</group>
```
## Usage
It is important to mention that some modification on CMakelist files of the crazyflie_tools, crazyflie-firmware and Crazyswam is modified in order to use this current folder organization. Running the original GitHub repository of the Crazyswam library might not work.
- Put all this packages in your src folder of the ROS workspace and build.
- Run the following command to execute the demonstration
If you run the formation with the mobile robots, make sure to setup the velocity control on the omniwheels. See the omniwheel_robot folder for more detail.
  - In terminal 1
```
roslaunch coordination_formation_control_pkg bringup_crazyflie_servers.launch
```

  - In terminal 2
```
roslaunch coordination_formation_control_pkg mission_controller.launch
```
In the dynamic_reconfigure gui, in order to activate the formation, the initialize start_formation must be set to true. Either the mantain_position or the update_waypoint must be set in order for the formation control to start.
- mantain_position: this is when performing formation around a fix point without flocking with 0 velocity. When the formation position goal is reached, the formation does not shutdown.
- update_waypoint: this is when performing flocking with with velocity other than zero. When the goal waypoint is reached, the formation automatically shutdowns for safety.  

- The current implementation logs the data of each drone seperately. To plot the result of the flight run the following command:

```
python plot_results.py ../results/experiment_1_formation_3_uav/result_crazyflie_id_logdata crazyflie_id N
```
  - id is the drone id-> 1,2,3... The id must match. N is the number of drones
An example for drone 1 is:

```
python plot_results.py ../results/experiment_1_formation_3_uav/result_crazyflie_1_logdata crazyflie_1 3
```
An example for drone 2 is:
```
python plot_results.py ../results/experiment_1_formation_3_uav/result_crazyflie_2_logdata crazyflie_2 crazyflie_2 3
```

## General observations
- The current implementation is only intended to have a maximum of two mobile robot. If you want to have more mobile robots, modify the clustering strategy in the mission_controller node.
- Drone formation can be done with a maximum of 5 drones.
```
typedef enum formation_matrix
{
  PAIR =2,
  TRIANGLE = 3,
  SQUARE = 4,
  PENTAGON = 5
}formation_matrix_t;
```
- The current implementation of the crazyswarm only allows 2 drones per crazyradio PA. Trying to control more drones with one crazyradio PA did not work.
- The localization system used in the drones(deck house) is quite noisy. It made it difficult to do roll pitch control on the crazyflie drone. It would be nice to include a filtering to improve accuracy.



## Future work
A detailed explanation on the possible future work to improve this project can be found in the masters thesis report.

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
