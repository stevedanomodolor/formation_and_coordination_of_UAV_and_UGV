import rosbag
import sys
import numpy as np
from matplotlib import pyplot as plt
import time

if(len(sys.argv) != 4):
    print ("invalid number of arguments:   " + str(len(sys.argv)))
    print ("should be 2: 'plot_results.py' and 'bagName'")
    print ("or just 1  : 'plot_results.py'")
    sys.exit(1)
else:
    bag_name = str(sys.argv[1]);
    robot_name = str(sys.argv[2])
    n_robots_logged = int(sys.argv[3])
    print(n_robots_logged)
    print("Plotting result from robot: " + robot_name + ", bag_name " + bag_name)
bag = rosbag.Bag(bag_name)
topic_name = robot_name+"result"
t_ = []
pose_data = []
vel_data = []
acc_data = []
command_pose_data = []
command_twist_data = []
input_data = []
input_formation_data = []
input_orientation_data = []
input_navigation_data = []
input_obstacle_data = []
input_integration_data = []
input_to_system_data = []

pose_row_ = [0] * n_robots_logged*2;
twist_row_ = [0] * n_robots_logged*2;
acc_row_ = [0] * n_robots_logged*2;


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
    input_data.append([msg.input.x, msg.input.y])
    input_formation_data.append([msg.input_formation.x, msg.input_formation.y])
    input_orientation_data.append([msg.input_orientation.x, msg.input_orientation.y])
    input_navigation_data.append([msg.input_navigation.x, msg.input_navigation.y])
    input_obstacle_data.append([msg.input_obstacle.x, msg.input_obstacle.y])
    input_integration_data.append([msg.input_integration.x, msg.input_integration.y])
    input_to_system_data.append([msg.input_to_system.x, msg.input_to_system.y])

# print(pose_data)

bag.close()

t_np = np.array(t_)
# print(pose_data_np)

# remove the offset
t_np = t_np -t_np[0]

pose_data_np = np.array(pose_data);
# print(pose_data_np)
vel_data_np = np.array(vel_data);
acc_data_np = np.array(acc_data);
x_list = range(0,n_robots_logged*2,2);
y_list = range(1,n_robots_logged*2,2);
x_list_np = np.array(pose_data_np[:,x_list]);
y_list_np = np.array(pose_data_np[:,y_list]);
cent_pose_np = np.vstack((np.mean(x_list_np, axis=1), np.mean(y_list_np,axis =1))).transpose()
cent_twist_np = np.vstack((np.mean(np.array(vel_data_np[:,x_list]), axis=1), np.mean(np.array(vel_data_np[:,y_list]),axis =1))).transpose()
command_pose_np = np.array(command_pose_data);
command_twist_np = np.array(command_twist_data);
input_data_np = np.array(input_data);
input_formation_data_np = np.array(input_formation_data);
input_orientation_data_np = np.array(input_orientation_data);
input_navigation_data_np = np.array(input_navigation_data);
input_obstacle_data_np = np.array(input_obstacle_data);
input_integration_data_np = np.array(input_integration_data);
input_to_system_data_np = np.array(input_to_system_data);

# Start plotting
####################### command vs centroid ###################################
fig1, axs1 = plt.subplots(nrows=2, ncols=2, figsize=(7, 7))
st = fig1.suptitle("Command swam state vc current swam state", fontsize="x-large")

# vx
axs1[0, 0].plot(t_np, command_pose_np[:,0])
axs1[0, 0].plot(t_np, cent_pose_np[:,0])
axs1[0, 0].set_ylabel("x(m)")
axs1[0, 0].set_xlabel("time(s)")
axs1[0, 0].legend(["command", "state"])


axs1[0, 1].plot(t_np, command_pose_np[:,1])
axs1[0, 1].plot(t_np, cent_pose_np[:,1])
axs1[0, 1].set_ylabel("y(m)")
axs1[0, 1].set_xlabel("time(s)")
axs1[0, 1].legend(["command", "state"])


axs1[1, 0].plot(t_np, command_twist_np[:,0])
axs1[1, 0].plot(t_np, cent_twist_np[:,0])
axs1[1, 0].set_ylabel("vx(m/s)")
axs1[1, 0].set_xlabel("time(s)")
axs1[1, 0].legend(["command", "state"])

axs1[1, 1].plot(t_np, command_twist_np[:,1])
axs1[1, 1].plot(t_np, cent_twist_np[:,1])
axs1[1, 1].set_ylabel("Vy(m/s)")
axs1[1, 1].set_xlabel("time(s)")
axs1[1, 1].legend(["command", "state"])

####################### Disance plot ###################################
fig2 = plt.figure()
ax2 = plt.axes()
st = fig2.suptitle("Distance plot", fontsize="x-large")
name_distance = [];
for i in range(n_robots_logged):
    for j in range(n_robots_logged):
        if i != j:
            name_distance.append("D" + str(i) + str(j))
            dist_vec_np = pose_data_np[:,[i*2,i*2+1]]-pose_data_np[:,[j*2,j*2+1]]
            dist_vec_np_norm = np.linalg.norm(dist_vec_np, axis =1)
            ax2.plot(t_np,dist_vec_np_norm)

ax2.legend(name_distance)
####################### Velocity plot ###################################
fig3, axs3 = plt.subplots(nrows=2, ncols=2, figsize=(7, 7))
st = fig3.suptitle("Velocity ", fontsize="x-large")
robot_names = []
for i in range(n_robots_logged):
    robot_names.append("robot_" + str(i+1))
    vx = vel_data_np[:,i*2];
    vy = vel_data_np[:,i*2+1]
    axs3[0, 0].plot(t_np,vx )
    axs3[0, 1].plot(t_np, vy)
    axs3[1, 0].plot(t_np, np.sqrt(np.multiply(vx,vx)+np.multiply(vy,vy)))

axs3[0, 0].set_ylabel("vx(m)")
axs3[0, 0].set_xlabel("time(s)")
axs3[0, 0].legend(robot_names)
axs3[0, 1].set_ylabel("vy(m)")
axs3[0, 1].set_xlabel("time(s)")
axs3[0, 1].legend(robot_names)
axs3[1, 0].set_ylabel("v(m/s)")
axs3[1, 0].set_xlabel("time(s)")
axs3[1, 0].legend(robot_names)


####################### Acceleration plot ###################################
fig4, axs4 = plt.subplots(nrows=2, ncols=2, figsize=(7, 7))
st = fig4.suptitle("Acceleration", fontsize="x-large")
robot_names = []
for i in range(n_robots_logged):
    robot_names.append("robot_" + str(i+1))
    ax = acc_data_np[:,i*2];
    ay = acc_data_np[:,i*2+1]
    axs4[0, 0].plot(t_np,ax )
    axs4[0, 1].plot(t_np, ay)
    axs4[1, 0].plot(t_np, np.sqrt(np.multiply(ax,ax)+np.multiply(ay,ay)))

axs4[0, 0].set_ylabel("ax(m)")
axs4[0, 0].set_xlabel("time(s)")
axs4[0, 0].legend(robot_names)
axs4[0, 1].set_ylabel("ay(m)")
axs4[0, 1].set_xlabel("time(s)")
axs4[0, 1].legend(robot_names)
axs4[1, 0].set_ylabel("a(m/s)")
axs4[1, 0].set_xlabel("time(s)")
axs4[1, 0].legend(robot_names)


# multple robot plot for comparision
####################### Inputs plot ###################################
fig5, axs5 = plt.subplots(nrows=6, ncols=2, figsize=(10, 10))
st = fig5.suptitle("Input values", fontsize="x-large")

axs5[0, 0].plot(t_np,input_data_np[:,0])
axs5[0, 1].plot(t_np,input_data_np[:,1])
axs5[1, 0].plot(t_np,input_formation_data_np[:,0])
axs5[1, 1].plot(t_np,input_formation_data_np[:,1])
axs5[2, 0].plot(t_np,input_orientation_data_np[:,0])
axs5[2, 1].plot(t_np,input_orientation_data_np[:,1])
axs5[3, 0].plot(t_np,input_navigation_data_np[:,0])
axs5[3, 1].plot(t_np,input_navigation_data_np[:,1])
axs5[4, 0].plot(t_np,input_obstacle_data_np[:,0])
axs5[4, 1].plot(t_np,input_obstacle_data_np[:,1])
axs5[5, 0].plot(t_np,input_integration_data_np[:,0])
axs5[5, 1].plot(t_np,input_integration_data_np[:,1])

axs5[0, 0].set_ylabel("a(m2/s)")
axs5[0, 0].set_xlabel("time(s)")
axs5[0, 0].set_title("input")
axs5[0, 1].set_ylabel("a(m2/s)")
axs5[0, 1].set_xlabel("time(s)")
axs5[0, 1].set_title("input")
axs5[1, 0].set_ylabel("a(m2/s)")
axs5[1, 0].set_xlabel("time(s)")
axs5[1, 0].set_title("input_formation")
axs5[1, 1].set_ylabel("a(m2/s)")
axs5[1, 1].set_xlabel("time(s)")
axs5[1, 1].set_title("input_formation")
axs5[2, 0].set_ylabel("a(m2/s)")
axs5[2, 0].set_xlabel("time(s)")
axs5[2, 0].set_title("input_orientation")
axs5[2, 1].set_ylabel("a(m2/s)")
axs5[2, 1].set_xlabel("time(s)")
axs5[2, 1].set_title("input_orientation")
axs5[3, 0].set_ylabel("a(m2/s)")
axs5[3, 0].set_xlabel("time(s)")
axs5[3, 0].set_title("input_navigation")
axs5[3, 1].set_ylabel("a(m2/s)")
axs5[3, 1].set_xlabel("time(s)")
axs5[3, 1].set_title("input_navigation")
axs5[4, 0].set_ylabel("a(m2/s)")
axs5[4, 0].set_xlabel("time(s)")
axs5[4, 0].set_title("input_obstacle")
axs5[4, 1].set_ylabel("a(m2/s)")
axs5[4, 1].set_xlabel("time(s)")
axs5[4, 1].set_title("input_obstacle")
axs5[5, 0].set_ylabel("a(m2/s)")
axs5[5, 0].set_xlabel("time(s)")
axs5[5, 0].set_title("input_integration")
axs5[5, 1].set_ylabel("a(m2/s)")
axs5[5, 1].set_xlabel("time(s)")
axs5[5, 1].set_title("input_integration")


####################### position plot ###################################
fig6 = plt.figure()
axs6 = plt.axes()
st = fig6.suptitle("Position plot", fontsize="x-large")
# plot workspace
axs6.set_xlim(-3, 3)
axs6.set_ylim(-2, 2)
# print(pose_data_np[:,0])
# print(pose_data_np[:,1])
# axs6.plot(pose_data_np[:,0],pose_data_np[:,1],"*")

for i in range(n_robots_logged):
    axs6.plot(pose_data_np[:,i*2],pose_data_np[:,i*2+1],".")
axs6.set_ylabel("y(m)")
axs6.set_xlabel("x(m)")
axs6.legend(robot_names)

# multple robot plot for comparision
####################### Direct command ###################################
fig6, axs6 = plt.subplots(nrows=2, ncols=2, figsize=(10, 10))
st = fig6.suptitle("Direct command to robot (velocity)", fontsize="x-large")

axs6[0, 0].plot(t_np,input_to_system_data_np[:,0])
axs6[0, 1].plot(t_np,input_to_system_data_np[:,1])
vx = input_to_system_data_np[:,0];
vy = input_to_system_data_np[:,1]
axs6[1, 0].plot(t_np,np.sqrt(np.multiply(vx,vx)+np.multiply(vy,vy)))

axs6[0, 0].set_ylabel("vx(m)")
axs6[0, 0].set_xlabel("time(s)")
axs6[0, 0].legend(robot_names)
axs6[0, 1].set_ylabel("vy(m)")
axs6[0, 1].set_xlabel("time(s)")
axs6[0, 1].legend(robot_names)
axs6[1, 0].set_ylabel("v(m/s)")
axs6[1, 0].set_xlabel("time(s)")
axs6[1, 0].legend(robot_names)
plt.show()
