#!/usr/bin/env python3
import logging
import numpy as np
from threading import Timer


import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander
logging.basicConfig(level=logging.ERROR)

# ros stuff 
import rospy
import time
from std_msgs.msg import String
from omniwheels_controller.msg import motorCommand
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from Buffer import Buffer

uri = uri_helper.uri_from_env(default='usb://0') # Connection-uri for crazyflie via USB

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class ParameterLogging:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """
    period_in_ms = 30

    def __init__(self, link_uri):
        
        # ros stuff
        self._pose_data_ = Pose()
        self._vel_data_ = Twist()
        self._acc_data_ = Vector3()
        # Buffer
        self.bs = 30
        self.pos_x_buf = Buffer(self.bs)
        self.pos_y_buf = Buffer(self.bs)
        self.pos_yaw_buf = Buffer(self.bs)
        self.vel_x_buf = Buffer(self.bs)
        self.vel_y_buf = Buffer(self.bs)
        self.vel_z_buf = Buffer(self.bs)
        # create publishers 
        self._position_pub = rospy.Publisher("pose", Pose,queue_size=100)
        self._velocity_pub = rospy.Publisher("vel", Twist,queue_size=100)
        self._acceleration_pub = rospy.Publisher("acc", Vector3,queue_size=100)

        robot_name = rospy.get_param("/name")
        self._position_offset = rospy.get_param("/position_offset")
        #
        print("Started " + robot_name + " Logging")
        self._cf = Crazyflie(rw_cache='./cache')
        print("Connectiong callbacks")
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self.R = rospy.get_param("/R")
        
        #try to connected to the crazyflie 
        self._cf.open_link(link_uri)
       

        


    def _connected(self, link_uri):
        print("Connected")
        #self.reset_estimator(self._cf)
        print("Reset estimation complete")

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='formation_test_position', period_in_ms=self.period_in_ms)
        self._lg_stab.add_variable('stateEstimate.yaw')
        self._lg_stab.add_variable('stateEstimateZ.x')
        self._lg_stab.add_variable('stateEstimateZ.y')
        self._lg_stab.add_variable('stateEstimateZ.z')
        print("Position configuraion finished")


        self._lg_est_vel = LogConfig(name='formation_test_velocity', period_in_ms=self.period_in_ms)
        self._lg_est_vel.add_variable('stateEstimateZ.vx')
        self._lg_est_vel.add_variable('stateEstimateZ.vy')
        self._lg_est_vel.add_variable('stateEstimateZ.rateYaw')
        
        print("velocity configuraion finished")
        
        self._lg_est_acc = LogConfig(name='formation_test_acceleration', period_in_ms=self.period_in_ms)
        self._lg_est_acc.add_variable('stateEstimateZ.ax')
        self._lg_est_acc.add_variable('stateEstimateZ.ay')
        self._lg_est_acc.add_variable('stateEstimateZ.az')
        
        print("velocity configuraion finished")

        try:
            self._cf.log.add_config(self._lg_stab)
            self._cf.log.add_config(self._lg_est_vel)
            self._cf.log.add_config(self._lg_est_acc)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._pos_log_data)
            self._lg_est_vel.data_received_cb.add_callback(self._vel_log_data)
            self._lg_est_acc.data_received_cb.add_callback(self._acc_log_data)
       

            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._log_error)
            self._lg_est_vel.error_cb.add_callback(self._log_error)
            self._lg_est_acc.error_cb.add_callback(self._log_error)
          
            # Start the logging
            self._lg_stab.start()
            self._lg_est_vel.start()
            self._lg_est_acc.start()
            print("Completed configuration")
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')
        
        
        # Start a timer to disconnect in 10s
        #t = Timer(5, self._cf.close_link)
        #t.start()
        #print("Timer started")


    def reset_estimator(self):
        self._cf.param.set_value('kalman.resetEstimation','1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '1')
        print("Waiting for position estimate")
        # self.wait_for_position_estimator()

    def wait_for_position_estimator(self):
        print('Waiting for estimator to find position...')
        log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
        log_config.add_variable('kalman.varPX', 'float')
        log_config.add_variable('kalman.varPY', 'float')
        log_config.add_variable('kalman.varPZ', 'float')

        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.001

        with SyncLogger(self._cf, log_config) as logger:
            for log_entry in logger:
                data = log_entry[1]
                var_y_history = [1000] * 10
                var_x_history = [1000] * 10
                var_z_history = [1000] * 10

                threshold = 0.001   
                # with SyncLogger(self._cf, self._lg_stab) as logger:
                while True:

                    var_x_history.append(data['kalman.varPX'])
                    var_x_history.pop(0)
                    var_y_history.append(data['kalman.varPY'])
                    var_y_history.pop(0)
                    var_z_history.append(data['kalman.varPZ'])
                    var_z_history.pop(0)


                    min_x = min(var_x_history)
                    max_x = max(var_x_history)
                    min_y = min(var_y_history)
                    max_y = max(var_y_history)
                    min_z = min(var_z_history)
                    max_z = max(var_z_history)

                    # print("{} {} {}".
                    #       format(max_x - min_x, max_y - min_y, max_z - min_z))

                    if (max_x - min_x) < threshold and (
                            max_y - min_y) < threshold and (
                            max_z - min_z) < threshold:
                        break

  

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _pos_log_data(self, timestamp, data, logconf):
        self.pos_x_buf.fill(data['stateEstimateZ.x']/1000.0)
        self.pos_y_buf.fill(data['stateEstimateZ.y']/1000.0)
        z = data['stateEstimateZ.z']/1000.0
        self.pos_yaw_buf.fill(data['stateEstimate.yaw'])
        yaw_omni = np.mod(self.pos_yaw_buf.get_mean()-30, 360);

       
        quat_val = get_quaternion_from_euler(0,0,np.deg2rad(yaw_omni))
        

        
        self._pose_data_.orientation.x = quat_val[0]
        self._pose_data_.orientation.y =quat_val[1]
        self._pose_data_.orientation.z = quat_val[2]
        self._pose_data_.orientation.w = quat_val[3] 
        self._pose_data_.position.x = self.pos_x_buf.get_mean() + np.sin(np.deg2rad(self.pos_yaw_buf.get_mean()))*self._position_offset;
        self._pose_data_.position.y = self.pos_y_buf.get_mean()- np.cos(np.deg2rad(self.pos_yaw_buf.get_mean()))*self._position_offset
        self._pose_data_.position.z = z;
        
             
        self._position_pub.publish(self._pose_data_)
       
    def _vel_log_data(self, timestamp,data, logconf):
        self.vel_x_buf.fill(data['stateEstimateZ.vx']/1000.0)
        self.vel_y_buf.fill(data['stateEstimateZ.vy']/1000.0)
        self.vel_z_buf.fill(data['stateEstimateZ.rateYaw']/1000.0)

        self._vel_data_.linear.x = self.vel_x_buf.get_mean()
        self._vel_data_.linear.y = self.vel_y_buf.get_mean()
        self._vel_data_.angular.z = self.vel_z_buf.get_mean()
        self._velocity_pub.publish(self._vel_data_)
        
    def _acc_log_data(self, timestamp,data, logconf):
        self._acc_data_.x = data['stateEstimateZ.ax']/1000.0
        self._acc_data_.y = data['stateEstimateZ.ay']/1000.0
        self._acc_data_.z = data['stateEstimateZ.az']/1000.0
        self._acceleration_pub.publish(self._acc_data_)
        
    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False
        
    def close(self):
        self._cf.close_link()



if __name__ == '__main__':
    rospy.init_node("crazyflie_logger_node")
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    try:
        le = ParameterLogging(uri)
        
    except rospy.ROSInterruptException:
        pass

 
