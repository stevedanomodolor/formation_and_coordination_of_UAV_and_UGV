%% test and ensure models works 


clear 
close all 
% load the models
agv_models = load(pwd + "/models/drone/second_order_model_drone.mat");
ugv_models = load(pwd + "/models/ugv/ugv_model.mat");


ugv.vx = ugv_models.d_sysx;
ugv.vy = ugv_models.d_sysy;
agv.vx =  agv_models.d_sysx;
agv.vy =  agv_models.d_sysy;
noise.vx = agv_models.std_errorx;
noise.vy = agv_models.std_errory;
dt = 0.01;
clear agv_models ugv_models
%%
% inital position
q0 = [0;0];
p0 = [0;0];
a0 = [0;0];

N = size(p0,1);
% create the drone object 
drone1 = droneSecondOrderModel(q0, p0,a0, N, agv, 1, false, true, noise);
drone1.display_state

% create ugv object
ugv1 = mobileRobotSingleIntegrator(q0, p0,N, ugv, 1,false,0.02, 0.02);
ugv1.display_state



t = 0:dt:8; % simulation time horizon
s = size(t,2);
% constant input  
u = ones(2,s) * 30;


%% start simulation
outputs_agv= [];
outputs_ugv= [];

for i = 1:s
   drone1.update_state(u(:,i));
   ugv1.update_state(u(:,i));

   o1 = drone1.get_state;
   o2 = ugv1.get_state;

   outputs_agv = [outputs_agv;o1.p(1) o1.p(2) o1.q(1) o1.q(2)];
   outputs_ugv = [outputs_ugv;o2.p(1) o2.p(2) o2.q(1) o2.q(2)];

end 

%%

figure 
subplot(2,2,1)
plot(t,outputs_agv(:,1))
subplot(2,2,2)
plot(t,outputs_agv(:,2))
subplot(2,2,3)
plot(t,outputs_agv(:,3))
subplot(2,2,4)
plot(t,outputs_agv(:,4))

figure 
subplot(2,2,1)
plot(t,outputs_ugv(:,1))
subplot(2,2,2)
plot(t,outputs_ugv(:,2))
subplot(2,2,3)
plot(t,outputs_ugv(:,3))
subplot(2,2,4)
plot(t,outputs_ugv(:,4))