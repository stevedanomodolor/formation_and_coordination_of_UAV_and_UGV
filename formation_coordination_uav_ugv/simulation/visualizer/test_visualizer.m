%% Test visualize function 

close all
clear all
file_path = '~/masters_thesis_stevedan/masters-thesis/code/control/simulation/simulation_results/test_logger/';
file_name = 'test_logger_1.csv';
file_ =[file_path file_name];

%% test the class and ensure it is working as expected
free_space = 1; 
split_rejoin = 2;
trajectory = 3;
obs_pos = [];
robot_naming = {'drone1', 'drone2'};

E1 = EnironmentSetupSimulation(free_space, obs_pos);
[fig, ax] = E1.showEnvironment;
n_data = 9;
visualize_results(file_,ax,2,n_data,robot_naming);
legend(ax)
