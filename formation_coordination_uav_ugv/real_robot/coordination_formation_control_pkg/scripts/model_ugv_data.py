#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import time
import numpy as np
import matplotlib.pyplot as plt

vel_cmd_vector = [[0.5 ,0]];
N = 1;

data_xy = []
c = 0
t = []

def velCallback(msg):
    global data_xy
    global c
    global t
    t = rospy.Time.now()
    tp = t.secs + t.nsecs*1e-9
    # print(c)
    if c < N:
        data_xy.append([tp, vel_cmd_vector[c][0], vel_cmd_vector[c][1],msg.linear.x, msg.linear.y])

def main():
    global data_xy
    global c
    global t
    rospy.init_node('model_ugv_data', anonymous =True)
    t = Header()
    pub = rospy.Publisher('omniwheel1/vel_cmd', Twist, queue_size=1);
    rospy.Subscriber("omniwheel1/vel", Twist, velCallback)
    rate = rospy.Rate(100)
    command = Twist()
    command.linear.x = vel_cmd_vector[c][0]
    command.linear.y = vel_cmd_vector[c][1]
    command.linear.z =0
    command.angular.x = 0
    command.angular.y = 0
    command.angular.z = 0
    t0 = time.time()
    timer = 0
    print(vel_cmd_vector[c][:])

    while not rospy.is_shutdown():

        if (timer > 30): #every 3 seconds
            c = c +1;
            if c == N:
                print("Stopping test")
                break;
            else:
                t0 = time.time()
                timer = 0;
                print(vel_cmd_vector[c][:])
                command.linear.x = vel_cmd_vector[c][0]
                command.linear.y = vel_cmd_vector[c][1]
        t1 = time.time()
        timer = t1-t0;
        # print(timer)
        pub.publish(command)
        rate.sleep()

    # convert array to number
    data_xy_np = np.array(data_xy);
    if (data_xy_np.size != 0):
        data_xy_np[:,0] = data_xy_np[:,0]- data_xy_np[0,0]
        # write it to a file
        print("Saving data to file")
        np.savetxt("data_omniwheelsx.csv",data_xy_np, delimiter="," )
        # plot results
        ####################### Direct command ###################################
        fig, (ax1, ax2) = plt.subplots(1, 2)
        fig.suptitle('Velocity omniwheels command vs current state')
        ax1.plot(data_xy_np[:,0], data_xy_np[:,1])
        ax1.plot(data_xy_np[:,0], data_xy_np[:,3])
        ax2.plot(data_xy_np[:,0], data_xy_np[:,2])
        ax2.plot(data_xy_np[:,0], data_xy_np[:,4])
        ax2.set_xlim([0,data_xy_np[-1,0] ])
        ax2.set_ylim([-0.1, 0.1])
        ax1.set_xlim([0,data_xy_np[-1,0] ])
        ax1.set_ylim([-0.1, 0.1])
        plt.show()






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
