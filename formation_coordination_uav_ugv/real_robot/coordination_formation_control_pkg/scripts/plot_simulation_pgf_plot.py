import numpy as np
from numpy import genfromtxt
import matplotlib
matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    # 'tick.labelsize': 0.05,   # large tick labels
    'ytick.labelsize':   'xx-small',
    'xtick.labelsize':   'xx-small',
    'pgf.rcfonts': False,
    "figure.autolayout": True,
    #  "pgf.preamble": [
    #     r"\usepackage[utf8x]{inputenc}",
    #     r"\usepackage[T1]{fontenc}",
    # ],
})
from matplotlib import pyplot as plt
result_folder_name = "/home/spot/masters_thesis_stevedan/formation_coordination_uav_ugv/simulation/simulation_results"
file_name = "/test_12_A/test_12_A.csv"
file_name_cluster = "/test_12_A/clusters.csv"

N = 3 # number of robots
N_uavs = 3;
N_ugvs = 2;
d_uav = 0.5;
add_obs = False;
obs_mat = [0,0,0.25]
raw_data = genfromtxt(result_folder_name+file_name, delimiter=',')[1:-1, :];
raw_data_cluster = genfromtxt(result_folder_name+file_name_cluster, delimiter=',');

rs = raw_data.shape[0]; # row size
print("Number of rows: " + str(rs))

# extract relevant data
time = raw_data[:,0];
position_command = raw_data[:,[1,2]];
velocity_command = raw_data[:,[3,4]];
position_x = np.zeros([rs,N]);
position_y = np.zeros([rs,N]);
velocity_x = np.zeros([rs,N]);
velocity_y = np.zeros([rs,N]);
ugv_position_x =  np.zeros([rs,N_ugvs]);
ugv_position_y =  np.zeros([rs,N_ugvs]);
cluster_x =  raw_data_cluster[:,[0,2]];
cluster_y =  raw_data_cluster[:,[1,3]];
for i in range(rs):
    for j in range(N):
        position_x[i,j] = raw_data[i,j*18+5]
        position_y[i,j] = raw_data[i,j*18+6]
        velocity_x[i,j] = raw_data[i,j*18+7]
        velocity_y[i,j] = raw_data[i,j*18+8]
    for p in range(N_ugvs):
        ugv_position_x[i,p] = raw_data[i,(N_uavs+p)*18+5]
        ugv_position_y[i,p] = raw_data[i,(N_uavs+p)*18+6]

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
    axis_traj.plot(cluster_x[0,:], cluster_y[0,:], ".",marker="h",markersize=12, markeredgecolor="black", markerfacecolor="black")
    axis_traj.plot(cluster_x[-1,:], cluster_y[-1,:], ".",marker="+",markersize=12, markeredgecolor="black", markerfacecolor="black")



#plot final configuration
axis_traj.plot(np.append(position_x[-1,:],position_x[-1,0]) , np.append(position_y[-1,:],position_y[-1,0]),"r")
if N_ugvs > 0:
    axis_traj.plot(np.append(ugv_position_x[-1,0],cluster_x[-1,0]) , np.append(ugv_position_y[-1,0],cluster_y[-1,0]),"k")
    axis_traj.plot(np.append(ugv_position_x[-1,1],cluster_x[-1,1]) , np.append(ugv_position_y[-1,1],cluster_y[-1,1]),"k")
#plot trajectories
color_array = ["c", "g","b", "y", "m", "C1", "C2"]
# plot the uavs
for i in range(N_uavs):
    c = color_array[i];
    axis_traj.plot(position_x[:,i], position_y[:,i], c,label="UAV_" + str(i+1))

for t in range(N_ugvs):
    c = color_array[t+N_uavs];
    c1 = color_array[t+N_uavs+2];
    axis_traj.plot(ugv_position_x[:,t], ugv_position_y[:,t],c+"--",label="UGV_" + str(t+1))
    axis_traj.plot(cluster_x[:,t], cluster_y[:,t],c1+"--",label="Cluster_" + str(t+1))

# plot the obstacle
if add_obs == True:
    obs1 = plt.Circle((obs_mat[0], obs_mat[1]), obs_mat[2]*2, color='g', clip_on=False, label="obstacle")
    axis_traj.add_patch(obs1)


# plot workspace
# axis_traj.set_xlim(-3.5, 0.5)
# axis_traj.set_ylim(-2, 2)

axis_traj.set_xlim(-3, 3)
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
    dist_vect2 =np.array([cluster_x[:,i],cluster_y[:,i]])
    label_name = "C"+str(i+1)+ "D"+str(i+1)
    ddd_ = dist_vect1-dist_vect2;
    ddd_ = ddd_.transpose()
    disdd = np.linalg.norm(ddd_,axis =1)
    axis_dist.plot(time, disdd, label = label_name)
# print(dist_vect)




axis_dist.set_ylabel("Distance(m)",fontsize="small")
axis_dist.set_xlabel("Time(s)",fontsize="small")
axis_dist.legend(fontsize="xx-small")
axis_dist.set_ylim(0, 2.5)
axis_dist.set_xlim(0, time[-1])
# fig_dist.set_size_inches(w=4, h=3)
fig_dist.set_size_inches(w=4, h=2.5)


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
axis_ori.plot(time,angle,label="Orientation")
do = np.ones([rs,1])*0;

axis_ori.plot(time, do, label="Orientation reference")

axis_ori.set_ylabel("Orienation(" + u"\N{DEGREE SIGN}" + ")",fontsize="small")
axis_ori.set_xlabel("Time(s)",fontsize="small")
axis_ori.legend(fontsize="xx-small")
# axis_ori.set_ylim(-180, 180)
# axis_ori.set_xlim(0, time[-1])
fig_ori.set_size_inches(w=4, h=2.5)



# plt.show()
# plt.style.use('seaborn')
## Expot file to pgf
# #
fig_traj.savefig("simulation_plot/type_5_trajectory_3_uavs_2_ugvs_coord.pgf",format='pgf')
fig_dist.savefig("simulation_plot/type_5_distance_3_uavs_2_ugvs_coord.pgf",format='pgf')
fig_nv.savefig("simulation_plot/type_5_navigation_3_uavs_2_ugvs.pgf",format='pgf')
fig_ori.savefig("simulation_plot/type_5_orientation_3_uavs_2_ugvs.pgf",format='pgf')
