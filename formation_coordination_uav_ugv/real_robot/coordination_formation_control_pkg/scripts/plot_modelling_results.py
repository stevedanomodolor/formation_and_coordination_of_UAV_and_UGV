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
result_folder_name = "/home/spot/masters_thesis_stevedan/formation_coordination_uav_ugv/simulation/models"
x_file_name = "/X_model.csv"
y_file_name =  "/Y_model.csv"


x_raw_data = genfromtxt(result_folder_name+x_file_name, delimiter=',')[1:-1, :];
y_raw_data = genfromtxt(result_folder_name+y_file_name, delimiter=',');



# extract relevant data
x_time = x_raw_data[:,0];
y_time = y_raw_data[:,0];

x_input = x_raw_data[:,1];
y_input = y_raw_data[:,1];

x_y = x_raw_data[:,2];
y_y = y_raw_data[:,2];

x_output = x_raw_data[:,3];
y_output = y_raw_data[:,3];


x_time_np = np.array(x_time);
y_time_np = np.array(y_time);

x_input_np = np.array(x_input);
y_input_np = np.array(y_input);

x_y_np = np.array(x_y);
y_y_np = np.array(y_y);

x_output_np = np.array(x_output);
y_output_np = np.array(y_output);

# print(ugv_position_y.shape)

############################# plot the trajectories ####################################
fig_x = plt.figure()
axis_x = plt.axes()
st = fig_x.suptitle("System indentification result: X model", fontsize="medium")

# add initial and end position
axis_x.plot(x_time, x_input, label = "Input")
axis_x.plot(x_time, x_y, label = "Experimental data")
axis_x.plot(x_time, x_output, label =  "Ouptut model")

# axis_traj.set_xlim(-3, 3)
# axis_traj.set_ylim(-1.5, 1.5)
axis_x.set_xlim(x_time[0], x_time[0-1])
axis_x.set_ylabel("x(m)",fontsize="small")
axis_x.set_xlabel("Time(s)",fontsize="small")
axis_x.legend(fontsize="small")
# fig_traj.set_size_inches(w=4, h=3)
fig_x.set_size_inches(w=4.5, h=3.5)

###########################
fig_y = plt.figure()
axis_y = plt.axes()
st = fig_y.suptitle("System indentification result: Y model", fontsize="medium")

# add initial and end position
axis_y.plot(y_time, y_input, label = "Input")
axis_y.plot(y_time, y_y, label = "Experimental data")
axis_y.plot(y_time, y_output, label = "Ouptut model")

axis_y.set_xlim(y_time[0], y_time[0-1])
# axis_traj.set_ylim(-1.5, 1.5)
axis_y.set_ylabel("y(m)",fontsize="small")
axis_y.set_xlabel("Time(s)",fontsize="small")
axis_y.legend(fontsize="small")
# fig_traj.set_size_inches(w=4, h=3)
fig_y.set_size_inches(w=4.5, h=3.5)


plt.show()
#
#
fig_x.savefig("simulation_plot/x_model.pgf",format='pgf')
fig_y.savefig("simulation_plot/y_model.pgf",format='pgf')
