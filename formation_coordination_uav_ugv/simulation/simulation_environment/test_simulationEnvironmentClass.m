close all
clear all 

%% test the class and ensure it is working as expected
free_space = 1; 
split_rejoin = 2;
trajectory = 3;
obs_pos = [];
E1 = EnironmentSetupSimulation(free_space, obs_pos);
[fig, ax] = E1.showEnvironment;
% plot(ax, 1,1,'r+', 'MarkerSize',10)

