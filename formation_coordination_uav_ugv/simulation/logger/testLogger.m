%% test logger class
clear all
close all


n_robots = 2;
file_path = '~/masters_thesis_stevedan/masters-thesis/code/control/simulation/simulation_results/test_logger/';
file_name = 'test_logger_1';

% generate random data
n_data = 9;
robot_naming = {'drone1', 'drone2'};
data_logger = logger(file_path, file_name, n_robots, n_data,robot_naming);
for i = 1:20
    i = i*0.1;
    t = i;
    cq = i*ones(2,n_robots);
    cp =2*i* ones(2,n_robots);
    q = 3*i*ones(2,n_robots);
    p = 4*i*ones(2,n_robots);
    a = 5*i*ones(2,n_robots);
    inp = 6*i*ones(2,n_robots);
    inpf = 7*i*ones(2,n_robots);
    inpo = 8*i*ones(2,n_robots);
    inpn = 9*i*ones(2,n_robots);
    inpor = 9*i*ones(2,n_robots);
    inpint = 9*i*ones(2,n_robots);

    data_logger.logData(t, cq,cp,q,p,a,inp,inpf,inpo,inpn,inpor,inpint);


end
data_logger.saveData
