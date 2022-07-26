%% test controller class
% Formation controller 
%% Configuration parameters
close all 
clear all
format long


robot_size = 0.15;
id = "d1";
ind_pos = 1; % index position in the distance matrix 
controller_gains.c1_alpha = 0.2;
controller_gains.c2_alpha = 1; % c2 gain are recoputed
controller_gains.c1_beta = 0.3;
controller_gains.c2_beta = 1;
controller_gains.c1_gamma = 0.25;
controller_gains.c2_gamma = 0.2;
controller_gains.c1_theta = 0.2;
controller_gains.c1_delta = 0.09;

form_param.dist = 0.5;
form_param.type = 3;
form_param.k = 7;
form_param.ratio = 0.8; % ratio between dist inter and obstacle
form_param.eps = 0.1;
form_param.a = 5;
form_param.b = 5;
form_param.h_alpha = 0.2;
form_param.h_beta = 0.9;
form_param.d_obs = 0;
form_param.nav_type = 1; % triangle
form_param.integrator = 2;
form_param.dt = 0.02;
% form_param.nav_type = 2; 
% form_param.nav_type = 2; 
% form_param.nav_type = 2; 

form_param.integrator = 2;
cooperation = 1;
%% demo state and goal

q0 = [1 1 2;
    -1 -1.1 -2];

p0 = [0 0 0;
     0 0 0];
ref.q = [0;
    0 ];

ref.p = [5;
     1 ];
obs = [1, 0,0
    -1.15,0,0;
    0.5,2,3];

ori = 0;




c1 = SwamControllerAlg(robot_size, id,ind_pos,form_param,controller_gains);
%c1.showFormation
[input, input_vec ] = c1.controller(q0,p0,ref,obs,ori,cooperation);
% 
% input
% input_vec
