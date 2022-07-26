%% Main executation formation control, template test noise 0.001 static

clear all
close all

%% General parameters
dt = 0.01; % integration time
n_ugv = 0;
n_agv  = 4;
n_robots = n_agv + n_ugv;
agv_size = 0.15;
ugv_size = 0.32;
N = 2; % working in 2d plane

%% configuration robot model
% load the models
agv_models = load(pwd + "/models/drone/second_order_model.mat");
ugv_models = load(pwd + "/models/ugv/ugv_model.mat");
ugv.vx = ugv_models.d_sysx;
ugv.vy = ugv_models.d_sysy;
agv.vx =  agv_models.d_sysx;
agv.vy =  agv_models.d_sysy;
noise.vx = 0.001;%agv_models.std_errorx;
noise.vy = 0.001;%agv_models.std_errory;
clear agv_models ugv_models

%% load simulation environment
free_space = 1;
obs_pos = [];
E1 = EnironmentSetupSimulation(free_space, obs_pos);
[fig_live_plot, ax_live_plot] = E1.showEnvironment;

%% Initialize data logger
file_path = '~/masters_thesis_stevedan/masters-thesis/code/control/simulation/simulation_results/test_2_A/';
file_name = 'test_2_A';
n_data = 9;
robot_naming = {'drone1','drone2','drone3','drone4'};
data_logger = logger(file_path, file_name, n_robots,n_data,robot_naming);
q_log = zeros(N,n_robots);
p_log = zeros(N,n_robots);
a_log = zeros(N,n_robots);
inp_log = zeros(N,n_robots);
inpf_log = zeros(N,n_robots);
inpo_log = zeros(N,n_robots);
inpn_log = zeros(N,n_robots);
inpori_log = zeros(N,n_robots);
inpint_log = zeros(N,n_robots);


%% Controller setup
controller_gains.c1_alpha = 0.2; 
controller_gains.c2_alpha = 1; % c2 gain are recmoputed
controller_gains.c1_beta = 0.3;
controller_gains.c2_beta = 1;
controller_gains.c1_gamma = 0.1;
controller_gains.c2_gamma = 0.2;
controller_gains.c1_theta = 0.2;
controller_gains.c1_delta = 0.09;
controller_gains.c2_delta = 0.0;
controller_gains.c3_delta = 0.00;
% Formation parameters
form_param.dist = 0.5;
form_param.type = 4;
form_param.k = 7;
form_param.ratio = 0.8; % ratio between dist inter and obstacle
form_param.eps = 0.1;
form_param.a = 5;
form_param.b = 5;
form_param.h_alpha = 0.2;
form_param.h_beta = 0.9;
form_param.d_obs = 0;
form_param.nav_type = 1; %
form_param.integrator = 2;
form_param.dt = 0.01;
form_param.int_max  =0.1;

controller_agv = SwamControllerAlg.empty(n_agv,0);
for i = 1:n_agv
    controller_agv(i) = SwamControllerAlg(agv_size, "d" + num2str(i),i,form_param,controller_gains);
end

%% Robot states
q_agv = [-1.3 -1.3 -1.80 -1.8;
    1  -1 -1 1];
% q_agv =  [1.7559    1.7559    1.3404    1.3597;
%     0.1549   -0.2746   -0.3040    0.2027];
p_agv = [0 0 0 0;
    0 0 0 0];
a_agv = [0 0 0 0;
    0 0 0 0];
