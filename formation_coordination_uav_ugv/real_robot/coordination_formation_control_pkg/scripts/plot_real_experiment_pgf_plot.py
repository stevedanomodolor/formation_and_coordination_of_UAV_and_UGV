import numpy as np
from numpy import genfromtxt
import matplotlib
import rosbag
import time
#
# matplotlib.use("pgf")
# matplotlib.rcParams.update({
#     "pgf.texsystem": "pdflatex",
#     'font.family': 'serif',
#     'text.usetex': True,
#     # 'tick.labelsize': 0.05,   # large tick labels
#     'ytick.labelsize':   'xx-small',
#     'xtick.labelsize':   'xx-small',
#     'pgf.rcfonts': False,
#     "figure.autolayout": True,
#     #  "pgf.preamble": [
#     #     r"\usepackage[utf8x]{inputenc}",
#     #     r"\usepackage[T1]{fontenc}",
#     # ],
# })
from matplotlib import pyplot as plt

### obtain information from topic
# basic configurations
result_folder_name = "/home/spot/masters_thesis_stevedan/formation_coordination_uav_ugv/real_robot/coordination_formation_control_pkg/results"
file_name = "/experiment_6_formation_3_uav_2_ugv/result_crazyflie_2_logdata.bag"
topic_name = "crazyflie_2" + "result"
bag = rosbag.Bag(result_folder_name + file_name)

N_uavs = 3
N_ugvs = 2
n_obstacles = 0
d_uav = 0.8;
d_ugv = 1;
ratio_uav = 0.8;
ratio_ugv = 0.8;
d_uav2obs = d_uav*ratio_uav;
d_ugv2obs = d_ugv*ratio_ugv;

# add_obs = False;
# obs_mat = [0,0,0.25]
t_ = []
pose_data = []
ugv_pose_data = []
ugv_cluster_data = []
vel_data = []
acc_data = []
obs_data = []
command_pose_data = []
command_twist_data = []
pose_row_ = [0] * N_uavs*2;
twist_row_ = [0] * N_uavs*2;
acc_row_ = [0] * N_uavs*2;
ugv_pose_row_ = [0] * N_ugvs*2;
ugv_cluster_pose_row_ = [0] * N_ugvs*2;

# converting data to numpy for easy plotting
for topics, msg, t in bag.read_messages(topics =[topic_name]):
    for ind,pose in enumerate(msg.current_pose):
        # if(ind == 2):
        #         print(pose.position.y)
        # # print(ind)
        # print(str(pose.position.x) + " " +str(pose.position.y))
        pose_row_[ind*2] = pose.position.x
        pose_row_[ind*2+1] = pose.position.y
        twist_row_[ind*2] = msg.current_twist[ind].linear.x
        twist_row_[ind*2+1] = msg.current_twist[ind].linear.y
        acc_row_[ind*2] = msg.current_acc[ind].linear.x
        acc_row_[ind*2+1] = msg.current_acc[ind].linear.y
        if N_ugvs >0:
            for ind, pose in enumerate(msg.ugv_current_pose):
                ugv_pose_row_[ind*2] = pose.position.x
                ugv_pose_row_[ind*2+1] = pose.position.y
                ugv_cluster_pose_row_[ind*2] = msg.ugv_cluster[ind].position.x
                ugv_cluster_pose_row_[ind*2+1] = msg.ugv_cluster[ind].position.y
    if  abs(pose_row_[ind]) > 0.00000001:
        # print(pose_row_)
        # print(pose_row_)
        t_.append(msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9);
        pose_data.append(pose_row_[:]);
        # print(pose_data[0][:])
        # print(pose_data[-1][:])
        # time.sleep(1);
        vel_data.append(twist_row_[:]);
        acc_data.append(acc_row_[:]);
        command_pose_data.append([msg.command_pose.x, msg.command_pose.y, msg.command_pose.theta]);
        command_twist_data.append([msg.command_twist.linear.x, msg.command_twist.linear.y]);
        if N_ugvs > 0:
            ugv_pose_data.append(ugv_pose_row_[:]);
            ugv_cluster_data.append(ugv_cluster_pose_row_[:]);

        if n_obstacles > 0:
            # print(msg.current_obstacle)
            obs_data.append([msg.current_obstacle[0],msg.current_obstacle[1],msg.current_obstacle[2]])
bag.close()






t_np = np.array(t_)
rs = t_np.shape[0];

# remove the offset
time = t_np -t_np[0]
pose_data_np = np.array(pose_data);
# print(pose_data_np)
vel_data_np = np.array(vel_data);
acc_data_np = np.array(acc_data);
x_list = range(0,N_uavs*2,2);
y_list = range(1,N_uavs*2,2);
position_x = np.array(pose_data_np[:,x_list]);
position_y = np.array(pose_data_np[:,y_list]);
velocity_x = np.array(vel_data_np[:,x_list]);
velocity_y = np.array(vel_data_np[:,y_list]);
position_command = np.array(command_pose_data);
velocity_command = np.array(command_twist_data);


if N_ugvs > 0:
    ugv_pose_data_np = np.array(ugv_pose_data);
    ugv_cluster_data_np = np.array(ugv_cluster_data);
    x_list_ugv = range(0,N_ugvs*2,2);
    y_list_ugv = range(1,N_ugvs*2,2);
    ugv_position_x = np.array(ugv_pose_data_np[:,x_list_ugv]);
    ugv_position_y = np.array(ugv_pose_data_np[:,y_list_ugv]);
    ugv_cluster_x = np.array(ugv_cluster_data_np[:,x_list_ugv]);
    ugv_cluster_y = np.array(ugv_cluster_data_np[:,y_list_ugv]);
if n_obstacles > 0:
    obstacle_position = np.array(obs_data)

    # print(obstacle_position)

# print(position_y)

# print(ugv_cluster_y)
# print(ugv_position_y)


# if N_ugvs > 0:
#     ...
#     # raw_data_cluster = genfromtxt(result_folder_name+file_name_cluster, delimiter=',');
#     # cluster_x =  raw_data_cluster[:,[0,2]];
#     # cluster_y =  raw_data_cluster[:,[1,3]];
#     # ugv_position_x =  np.zeros([rs,N_ugvs]);
#     # ugv_position_y =  np.zeros([rs,N_ugvs]);






# for i in range(rs):
#     for j in range(N):
#         position_x[i,j] = raw_data[i,j*18+5]
#         position_y[i,j] = raw_data[i,j*18+6]
#         velocity_x[i,j] = raw_data[i,j*18+7]
#         velocity_y[i,j] = raw_data[i,j*18+8]
#     if N_ugvs > 0:
#         for p in range(N_ugvs):
#             ugv_position_x[i,p] = raw_data[i,(N_uavs+p)*18+5]
#             ugv_position_y[i,p] = raw_data[i,(N_uavs+p)*18+6]

# print(ugv_position_x.shape)
# print(ugv_position_y.shape)

############################# plot the trajectories ####################################
fig_traj = plt.figure()
axis_traj = plt.axes()

# add initial and end position
axis_traj.plot(position_x[0,:], position_y[0,:], ".",marker="D",markersize=8, markeredgecolor="black", markerfacecolor="black")
axis_traj.plot(position_x[-1,:], position_y[-1,:], ".",marker="*",markersize=10, markeredgecolor="black", markerfacecolor="black")

if N_ugvs > 0:
    axis_traj.plot(ugv_position_x[0,:], ugv_position_y[0,:], ".",marker="s",markersize=12, markeredgecolor="black", markerfacecolor="black")
    axis_traj.plot(ugv_position_x[-1,:], ugv_position_y[-1,:], ".",marker="p",markersize=12, markeredgecolor="black", markerfacecolor="black")
    # axis_traj.plot(ugv_cluster_x[0,:], ugv_cluster_y[0,:], ".",marker="h",markersize=12, markeredgecolor="black", markerfacecolor="black")
    # axis_traj.plot(ugv_cluster_x[-1,:], ugv_cluster_y[-1,:], ".",marker="+",markersize=12, markeredgecolor="black", markerfacecolor="black")



#plot final configuration
# axis_traj.plot(np.append(position_x[-1,:],position_x[-1,0]) , np.append(position_y[-1,:],position_y[-1,0]),"r")
if N_ugvs > 0:
    for i in range(N_ugvs):
        axis_traj.plot(np.append(ugv_position_x[-1,i],ugv_cluster_x[-1,i]) , np.append(ugv_position_y[-1,i],ugv_cluster_y[-1,i]),"k")
        axis_traj.plot(np.append(ugv_position_x[-1,i],ugv_cluster_x[-1,i]) , np.append(ugv_position_y[-1,i],ugv_cluster_y[-1,i]),"k")
#plot trajectories
color_array = ["c", "g","b", "y", "m", "C1", "C2"]
# plot the uavs
# for i in range(N_uavs):
#     c = color_array[i];
#     axis_traj.plot(position_x[:,i], position_y[:,i], c,label="UAV_" + str(i+1))

for t in range(N_ugvs):
    c = color_array[t+N_uavs];
    c1 = color_array[t+N_uavs+2];
    # axis_traj.plot(ugv_position_x[:,t], ugv_position_y[:,t],c+"--",label="UGV_" + str(t+1))
    axis_traj.plot(ugv_cluster_x[:,t], ugv_cluster_y[:,t],c1+"--",label="Cluster_" + str(t+1),marker="*",markersize=10)

# plot the obstacle
if n_obstacles >0:
    for i in range(obstacle_position.shape[0]):
        obs1 = plt.Circle((obstacle_position[i,0], obstacle_position[i,1]), 0.005, color='g', label = "Obstacle")#, clip_on=False)
        axis_traj.add_patch(obs1)


# plot workspace
# axis_traj.set_xlim(-3.5, 0.5)
# axis_traj.set_ylim(-2, 2)

axis_traj.set_xlim(-1.5, 2.5)
axis_traj.set_ylim(-1.5, 1.5)
axis_traj.set_ylabel("y(m)",fontsize="small")
axis_traj.set_xlabel("x(m)",fontsize="small")
axis_traj.legend(fontsize="small")
# fig_traj.set_size_inches(w=4, h=3)
fig_traj.set_size_inches(w=4.5, h=3.5)

############################# plot the distances ####################################
# list to prevent prinitng all distances
fig_dist = plt.figure()
axis_dist = plt.axes()
st = fig_dist.suptitle("Inter-agent distance", fontsize="medium")

# desired distance
if N_uavs == 4:
    id = np.sqrt(d_uav*d_uav*2);
    dd_id = np.ones([rs,1])*id;
    dd = np.ones([rs,1])*d_uav;
    axis_dist.plot(time, dd, label="Distance reference")
    axis_dist.plot(time, dd_id, label="Diagonal Distance reference")
    d_matrix = np.array([[0, 1, 1, 1],
                         [0, 0, 1, 1],
                         [0, 0, 0, 1],
                         [0, 0, 0, 0]])
elif N_uavs == 3:
    dd = np.ones([rs,1])*d_uav;
    axis_dist.plot(time, dd, label="Distance reference")
    d_matrix = np.array([[0, 1, 1],
                         [0, 0, 1],
                         [0, 0, 0]])
elif N_uavs == 5:
        id = d_uav + 2*d_uav*np.cos(np.deg2rad(72))
        dd_id = np.ones([rs,1])*id;
        dd = np.ones([rs,1])*d_uav;
        axis_dist.plot(time, dd, label="Distance reference")
        axis_dist.plot(time, dd_id, label="Diagonal Distance reference")
        d_matrix = np.array([[0, 1, 1,1,1],
                             [0, 0, 1,1,1],
                             [0, 0, 0,1,1],
                             [0, 0, 0,0,1],
                             [0, 0, 0,0,0]])
#plot the desired distance
if N_ugvs > 0:
    d2c = np.ones([rs,1])*d_ugv;
    axis_dist.plot(time, d2c, label="Distance to cluster reference")

# Plot interagent distance
dist_vect = np.array([rs,2])
for i in range(N_uavs):
    for j in range(N_uavs):
        if d_matrix[i,j] == 1:
            label_name = "D"+str(i+1)+str(j+1)
            dist_vect1 =np.array([position_x[:,i],position_y[:,i]])
            dist_vect2 =np.array([position_x[:,j],position_y[:,j]])
            ddd_ = dist_vect1-dist_vect2;
            ddd_ = ddd_.transpose()
            disdd = np.linalg.norm(ddd_,axis =1)
            axis_dist.plot(time, disdd, label = label_name)

# distance to cluster
for i in range(N_ugvs):
    dist_vect1 =np.array([ugv_position_x[:,i],ugv_position_y[:,i]])
    dist_vect2 =np.array([ugv_cluster_x[:,i],ugv_cluster_y[:,i]])
    label_name = "C"+str(i+1)+ "D"+str(i+1)
    ddd_ = dist_vect1-dist_vect2;
    ddd_ = ddd_.transpose()
    disdd = np.linalg.norm(ddd_,axis =1)
    axis_dist.plot(time, disdd, label = label_name)
# print(dist_vect)




axis_dist.set_ylabel("Distance(m)",fontsize="small")
axis_dist.set_xlabel("Time(s)",fontsize="small")
axis_dist.legend(fontsize="xx-small")
axis_dist.set_ylim(0, 1.5)
axis_dist.set_xlim(0, time[-1])
# fig_dist.set_size_inches(w=4, h=3)
fig_dist.set_size_inches(w=4, h=2.5)


#########################################Distance obstacle plot #############################

fig_dist_obs = plt.figure()
axis_dist_obs = plt.axes()
st = fig_dist_obs.suptitle("Obstacle distances", fontsize="medium")

if n_obstacles > 0:
    ddu = np.ones([rs,1])*d_uav2obs;
    axis_dist_obs.plot(time, ddu, label="Distance to obstacle reference UAV")
    for i in range(N_uavs):
        dist_vect1 =np.array([position_x[:,i],position_y[:,i]])
        dist_vect2 =np.array([obstacle_position[:,0],obstacle_position[:,1]])
        label_name = "DUAV" +str(i+1)+"-"+ "O1"
        ddd_ = dist_vect1-dist_vect2;
        ddd_ = ddd_.transpose()
        disdd = np.linalg.norm(ddd_,axis =1)
        axis_dist_obs.plot(time, disdd, label = label_name)

if N_ugvs > 1:
    ddu_ = np.ones([rs,1])*d_ugv2obs;
    axis_dist_obs.plot(time, ddu_, label="Distance to obstacle reference UGV")
    # this is inteded for only two ugvs todo, change if necessary
    dist_vect1 =np.array([ugv_position_x[:,0],ugv_position_y[:,0]])
    dist_vect2 =np.array([ugv_position_x[:,1],ugv_position_y[:,1]])
    # print(dist_vect1)
    label_name = "DUGV1" +"-"+ "DUGV2"
    ddd_ = dist_vect1-dist_vect2;
    ddd_ = ddd_.transpose()
    disdd = np.linalg.norm(ddd_,axis =1)
    # print(disdd)
    axis_dist_obs.plot(time, disdd, label = label_name)


axis_dist_obs.set_ylabel("Distance(m)",fontsize="small")
axis_dist_obs.set_xlabel("Time(s)",fontsize="small")
axis_dist_obs.legend(fontsize="xx-small")
# axis_dist_obs.set_ylim(0, 1.5)
axis_dist_obs.set_xlim(0, time[-1])
# fig_dist.set_size_inches(w=4, h=3)
fig_dist_obs.set_size_inches(w=4, h=2.5)




####################### command vs centroid ###################################
fig_nv, axis_nv = plt.subplots(nrows=2, ncols=2)
# st = fig_nv.suptitle("Absolute Error: Navigation goal vs current centroid", fontsize="small")

position_centroid = np.array([np.mean(position_x,axis=1), np.mean(position_y,axis=1)]).transpose()
velocity_centroid = np.array([np.mean(velocity_x,axis=1), np.mean(velocity_y,axis=1)]).transpose()
# fig.supxlabel('fig.supxlabel')
# fig_.tight_layout()
# print(position_centroid[:,0])
# vx
epsilon = 1e-8
axis_nv[0, 0].plot(time, ((position_command[:,0])-(position_centroid[:,0])))
# axis_nv[0, 0].plot(time, position_centroid[:,0])
axis_nv[0, 0].set_ylabel("Absolute error(m)",fontsize="x-small")
axis_nv[0, 0].set_xlabel("time(s)",fontsize="x-small")
# axis_nv[0, 0].legend(["command", "state"])
axis_nv[0, 0].set_xlim(0, time[-1])
axis_nv[0, 0].set_title("Position-x",fontsize="x-small")
# x_ticks = np.arange(time[0],time[-1],5).tolist()
# axis_nv[0, 0].set_xticklabels(time, rotation=0, fontsize="x-small")
# # axis_nv[0, 0].set_yticklabels(y_ticks, rotation=0, fontsize="x-small")


axis_nv[0, 1].plot(time, (position_command[:,1]-position_centroid[:,1]))
# axis_nv[0, 1].plot(time, position_centroid[:,1])
axis_nv[0, 1].set_ylabel("Absolute error(m)",fontsize="x-small")
axis_nv[0, 1].set_xlabel("time(s)",fontsize="x-small")
# axis_nv[0, 1].legend(["command", "state"],fontsize="small")
axis_nv[0, 1].set_xlim(0, time[-1])
axis_nv[0, 1].set_title("Position-y",fontsize="x-small")
# axis_nv[0, 1].tick_params(axis='x', labelsize=1)


axis_nv[1, 0].plot(time, (velocity_command[:,0]-velocity_centroid[:,0]))
# axis_nv[1, 0].plot(time, velocity_centroid[:,0])
axis_nv[1, 0].set_ylabel("Absolute error(m)",fontsize="x-small")
axis_nv[1, 0].set_xlabel("time(s)",fontsize="x-small")
# axis_nv[1, 0].legend(["command", "state"],fontsize="small")
axis_nv[1, 0].set_xlim(0, time[-1])
axis_nv[1, 0].set_title("Velocity-x",fontsize="x-small")
# axis_nv[1, 0].tick_params(axis='both', labelsize=5)


axis_nv[1, 1].plot(time, (velocity_command[:,1]-velocity_centroid[:,1]))
# axis_nv[1, 1].plot(time, velocity_centroid[:,1])
axis_nv[1, 1].set_ylabel("Absolute error(m)",fontsize="x-small")
axis_nv[1, 1].set_xlabel("time(s)",fontsize="x-small")
# axis_nv[1, 1].legend(["command", "state"],fontsize="small")
axis_nv[1, 1].set_xlim(0, time[-1])
axis_nv[1, 1].set_title("Velocity-y",fontsize="x-small")
# axis_nv[1, 0].tick_params(axis='both', labelsize=5)

fig_nv.set_size_inches(w=4, h=2.5)

####################### Orientation ###################################

fig_ori = plt.figure()
axis_ori  = plt.axes()

robot_1 = np.array([position_x[:,0],position_y[:,0] ]).transpose()
orien_vector = position_centroid -robot_1
angle = np.zeros([rs,1])
# print(orien_vector[4,:])
for i in range(rs):

    x = np.array([orien_vector[i,0],0])
    y = np.array([orien_vector[i,1],0])
    angle[i] = np.arctan(orien_vector[i,1]/orien_vector[i,0])*180.0/np.pi
    if i < 10:
        # print(i)
        if angle[i] < 0:
            angle[i] = - angle[i]
axis_ori.plot(time,angle,label="Orientation")
do = np.ones([rs,1])*0;

axis_ori.plot(time, do, label="Orientation reference")

axis_ori.set_ylabel("Orienation(" + u"\N{DEGREE SIGN}" + ")",fontsize="small")
axis_ori.set_xlabel("Time(s)",fontsize="small")
axis_ori.legend(fontsize="xx-small")
axis_ori.set_ylim(-20, 90)
axis_ori.set_xlim(0, time[-1])
fig_ori.set_size_inches(w=4, h=2.5)



plt.show()
# plt.style.use('seaborn')
## Expot file to pgf
# # #
# fig_traj.savefig("result_plots/exp8_formation_3_uav_trajectory.pgf",format='pgf')
# fig_dist.savefig("result_plots/exp8_formation_3_uav_distance.pgf",format='pgf')
# fig_nv.savefig("result_plots/exp8_formation_3_uav_navigation.pgf",format='pgf')
# fig_ori.savefig("result_plots/exp8_formation_3_uav_orientation.pgf",format='pgf')
# fig_dist_obs.savefig("result_plots/exp8_formation_3_uav_obstacle.pgf",format='pgf')
