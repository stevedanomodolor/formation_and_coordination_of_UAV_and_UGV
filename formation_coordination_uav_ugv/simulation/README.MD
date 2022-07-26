# Stevedan Master's thesis Formation and coordination of UAV and UGV.
This folder contains the implementation of the formation and coordination control on in simulation.

## General info
The implementation of the simulation has the same structure as the real robot but is written entirely in MATLAB. It automatically generates logs and plots automatically in the location specified in the main file location in the formation_tests folder.
```
file_path = '~/masters_thesis_stevedan/formation_coordination_uav_ugv/simulation/simulation_results/test_12_A/';
file_name = 'test_12_A';
```

## Usage
- All main file of execution are found in the formation_tests folder. All configuration is done in this files and it is quite straight forward.

## Observations
- Each folder contains a test file to ensure that the implementation work. To understand for example how to add the formation controller, the file test_controller in the controller folder shows you how to.
- Currently, the current implementation requires a definition of the environment in which the robot would have to work. An example of definition is shown below and also in the file test_simulationEnvironmentClass in the simulation_environment folder. There are three possibilities: 1-free_space, 2-obstacle_space and 3- trajectory.
```
% load simulation environment
free_space = 1;
split_rejoin = 2;
obs_pos = [];
E1 = EnironmentSetupSimulation(free_space, obs_pos);
% obs_pos = E1.obstacle_matrix;
[fig_live_plot, ax_live_plot] = E1.showEnvironment;
```
- Some improvement are needed in how the obstacle is set in the EnironmentSetupSimulation class.

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
