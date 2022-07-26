#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String
from omniwheels_controller.msg import motorCommand
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math
import numpy as np
from SimplePI import SimplePI

# Dynamixel packages
from dynamixel.model.xm430_w210_t_r import XM430_W210_T_R
import dynamixel.channel
import time


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians
class omniWheelvelocityController:
        def __init__(self):
                # Load the servos
                self.servos = self.load_servos()
                self.MAX_WHEEL_SPEED =310

                self.motor_command = [0,0,0];
                self.send_command = False;
                self.vel_cmd_arrived = False;
                self.motor_command[0] = 0;
                self.motor_command[1] = 0;
                self.motor_command[2] = 0;
                self.vel_cmd = Twist()
                self.vel_cmd.linear.x = 0;
                self.vel_cmd.linear.y = 0;
                self.vel_cmd.angular.z = 0;
                self._current_vel = Twist()
                self._current_pose = Pose()
                self.vel_callback_initiated  = False;
                self.pose_callback_initiated = False;
                self.R = rospy.get_param("/R")# Distance between center of robot and wheels
                self.a1 = 2*np.pi/3 # Angle between x axis and first wheel
                self.a2 = 4*np.pi/3  # Angle between x axis and second wheel
                self.r = rospy.get_param("/r") # Wheel radius. Has been fudge-factored because the actual velocity of the wheels did not align with the set-points.
                kp_x = rospy.get_param("/kp_x")
                ki_x = rospy.get_param("/ki_x")
                kp_y = rospy.get_param("/kp_y")
                ki_y = rospy.get_param("/ki_y")                
                kp_z = rospy.get_param("/kp_z")
                ki_z = rospy.get_param("/ki_z")   

                # get ros param value
                robot_name = rospy.get_param("/name");




                # subscriber
                #rospy.Subscriber("motor_command", motorCommand, self.motorVelocityCallback)
                # Velocity command node
                rospy.Subscriber("vel_cmd", Twist, self.velocityCommandCallback)
                # pose and velocity values
                ss = rospy.get_param("/name")
                ss_pose = "pose";
                ss_vel= "vel";
                rospy.Subscriber(ss_pose, Pose, self.poseCallback)
                rospy.Subscriber(ss_vel, Twist, self.velCallback)





                time.sleep(1) # Wait for connection to work
                rospy.loginfo("Starting controller: " +  robot_name)
                self.motor_command[0] = 0
                self.motor_command[1] = 0
                self.motor_command[2] = 0
                print("Starting main loop")
                rate_hz = rospy.get_param("/rate_hz")
                rate = rospy.Rate(rate_hz) # 10hz
                dt = 1.0/float(rate_hz);
                self.pi_x = SimplePI(kp_x,ki_x,dt)         # Create a PI-controller 
                self.pi_y = SimplePI(kp_y,ki_y,dt)         # Create a PI-controller 
                self.pi_z = SimplePI(kp_y,ki_y,dt)         # Create a PI-controller                 

                # main loop
                while not rospy.is_shutdown():
                        if self.vel_cmd_arrived == True and self.vel_callback_initiated == True and self.pose_callback_initiated==True:
                                # compute motor values from velocity command
                                roll, pitch,yaw = euler_from_quaternion(self._current_pose.orientation.x,self._current_pose.orientation.y,self._current_pose.orientation.z,self._current_pose.orientation.w)
                                vel_ref = np.array([self.vel_cmd.linear.x, self.vel_cmd.linear.y, self.vel_cmd.angular.z]) 
                                error = vel_ref - np.array([self._current_vel.linear.x, self._current_vel.linear.y, self._current_vel.angular.z]);
                                vel_x = vel_ref[0] + self.pi_x.update_control(error[0])
                                vel_y = vel_ref[1] + self.pi_y.update_control(error[1])
                                vel_z = vel_ref[2] + self.pi_z.update_control(error[2])
                                xdot = np.array([vel_x[0],vel_y[0],vel_z[0]])
                                ph = self.vel2motor(xdot, (yaw));
                                #print(np.rad2deg(yaw))
                                #print(error)
                                #print(xdot)
                                
                                self.motor_command[0] = ph[0]
                                self.motor_command[1] = ph[1]
                                self.motor_command[2] = ph[2]
                                #print(error_x)
                                self.vel_cmd_arrived = False

                        else:
                                self.motor_command[0] = 0
                                self.motor_command[1] = 0
                                self.motor_command[2] = 0
                                

                        # Send command to the motors
                        for (j,s) in enumerate(self.servos):
                                w_c_ = self.motor_command[j]
                                w_c = self.saturate(w_c_, self.MAX_WHEEL_SPEED);
                                
                                #print(str(w_c_) + ": " + str(w_c))
                                s.goal_velocity.write(round(w_c))#self.motor_command[j]))
                                #print(round(w_c))
                        rate.sleep()


        def load_servos(self):
                """
                Servo load function, based on the dynamixel implementation of Anders Blomdell.
                """
                # The speed and channel numbers can be found and checked via the dynamixel software "dynamixel wizard".
                # The device can be found by e.g. lsusb, and is currently adapted for the three-color robots.

                channel = dynamixel.channel.Channel(speed=57600,device='/dev/ttyACM0')
                servos = [ XM430_W210_T_R(channel, 1),
                XM430_W210_T_R(channel, 2),
                XM430_W210_T_R(channel, 3) ]
                for s in servos:
                        s.torque_enable.write(0)
                        print(s.model_number.read(), s.id.read())
                        s.operating_mode.write(1)
                        s.bus_watchdog.write(0) # Clear old watchdog error
                        s.bus_watchdog.write(100) # 2 second timeout
                        s.torque_enable.write(1)

                return servos

        def motorVelocityCallback(self, data):
                self.send_command = True
                self.motor_command[0] = data.data[0];
                self.motor_command[1] = data.data[1];
                self.motor_command[2] = data.data[2];
        def velocityCommandCallback(self,data):
                self.vel_cmd_arrived = True
                self.vel_cmd = data

        def velCallback(self, data):
                self._current_vel = data;
                self.vel_callback_initiated = True;

        def poseCallback(self, data):
                self._current_pose = data;
                self.pose_callback_initiated = True;
        def vel2motor(self,xdot, ang):
                M =  -1/self.r*np.array([[-np.sin(ang), np.cos(ang), self.R ],[-np.sin(ang+self.a1), np.cos(ang+self.a1), self.R],[-np.sin(ang+self.a2), np.cos(ang+self.a2), self.R]])
                return M.dot(xdot)
                #return M.dot(xdot)
        def saturate(self, command, limit):
            if command > limit:
                return limit
            elif command < -limit:
                return -limit
            else:
                return command
if __name__ == '__main__':
        rospy.init_node("omni_wheel_controller_node")
        try:
                omniwheel1 = omniWheelvelocityController()
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("ERROR : "+str(e))
