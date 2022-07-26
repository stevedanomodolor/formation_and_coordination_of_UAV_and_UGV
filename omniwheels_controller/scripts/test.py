import logging
import time
from threading import Timer
from typing import Set
import numpy as np
import matplotlib.pyplot as plt
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander
uri = uri_helper.uri_from_env(default='usb://0') # Connection-uri for crazyflie via USB


drone_connected = False

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

sequence = [
    (0.0, 0.3,(1.0-0.5),0.0),
    (0.0, 0.0,(1.0-0.5),0.0),
    (0.0,-0.3,(1.0-0.5),0.0),
    (0.0, -0.3,(1.5-0.5),0.0),
    (0.0, 0.0,(1.5-0.5),0.0),
    (0.0, 0.3,(1.5-0.5),0.0),
    (0.0, 0.3,(2.0-0.5),0.0),
    (0.0, 0.0,(2.0-0.5),0.0),
    (0.0,-0.3,(2.0-0.5),0.0),
    (0.0, 0.0,(1.0-0.5),0.0),
    (0.0, 0.0,0.4,0.0)

]


current_setpoint = [0,0,0,0]
setpoint_list = np.array([[0,0,0,0]])
current_pose_list = np.array([[0,0,0,0]])
stop = False


class ParameterLogging:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """
    period_in_ms = 20

    def __init__(self, link_uri,cf):
        """ Initialize and run the example with the specified link_uri """

        # self._cf = Crazyflie(rw_cache='./cache')
        self._cf = cf
        print("Connectiong callbacks")
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
       


    def _connected(self, link_uri):
        global drone_connected
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)
        drone_connected = True
        
        self.reset_estimator(cf)
        print("Reset estimation complete")

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=self.period_in_ms)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        # self._lg_stab.add_variable('stabilizer.roll', 'float')
        # self._lg_stab.add_variable('stabilizer.pitch', 'float')
        # self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        self._lg_kal_pos = LogConfig(name='Kalman Position', period_in_ms=self.period_in_ms)
        self._lg_kal_pos.add_variable('kalman.stateX', 'float')
        self._lg_kal_pos.add_variable('kalman.stateY', 'float')
        self._lg_kal_pos.add_variable('kalman.stateZ', 'float')

        # self._lg_kal_vel = LogConfig(name='Kalman Velocity', period_in_ms=self.period_in_ms)
        # self._lg_kal_vel.add_variable('kalman.statePX', 'float')
        # self._lg_kal_vel.add_variable('kalman.statePY', 'float')
        # self._lg_kal_vel.add_variable('kalman.statePZ', 'float')

        self._lg_kal_alt = LogConfig(name='Kalman Altitude', period_in_ms=self.period_in_ms)
        self._lg_kal_alt.add_variable('kalman.q0', 'float')
        self._lg_kal_alt.add_variable('kalman.q1', 'float')
        self._lg_kal_alt.add_variable('kalman.q2', 'float')
        self._lg_kal_alt.add_variable('kalman.q3', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            self._cf.log.add_config(self._lg_kal_pos)
            # self._cf.log.add_config(self._lg_kal_vel)
            self._cf.log.add_config(self._lg_kal_alt)


            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_kal_pos.data_received_cb.add_callback(self._pos_log_data)
            # self._lg_kal_vel.data_received_cb.add_callback(self._vel_log_data)
            self._lg_kal_alt.data_received_cb.add_callback(self._att_log_data)

            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._log_error)
            self._lg_kal_pos.error_cb.add_callback(self._log_error)
            # self._lg_kal_vel.error_cb.add_callback(self._log_error)
            self._lg_kal_alt.error_cb.add_callback(self._log_error)

            # Start the logging
            self._lg_stab.start()
            self._lg_kal_pos.start()
            # self._lg_kal_vel.start()
            self._lg_kal_alt.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')
        
        self.reset_estimator()
        print("Finished reset estimator")
        # Start a timer to disconnect in 10s
        # t = Timer(5, self._cf.close_link)
        # t.start()


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

    def _stab_log_data(self, timestamp, data, logconf):
        global current_setpoint
        global current_pose_list
        global setpoint_list
        global stop
        self.stab_att = np.r_[data['stateEstimate.x'],
                              data['stateEstimate.y'],
                              data['stateEstimate.z']]

        current_pose_list =np.append(current_pose_list,[[self.stab_att[0],self.stab_att[1],self.stab_att[2],0]], axis=0)
        setpoint_list = np.append(setpoint_list,[current_setpoint], axis=0) 
        print(current_setpoint)
        # """Callback from a the log API when data arrives"""
        # print(f'[{timestamp}][{logconf.name}]: ', end='')
        # for name, value in data.items():
        #     print(f'{name}: {value:3.3f} ', end='')
        # print()
    def _pos_log_data(self, timestamp,data, logconf):
        # global current_setpoint
        # global current_pose_list
        # global setpoint_list
        # global stop
        self.pos =   np.r_[data['kalman.stateX'],
                           data['kalman.stateY'],
                           data['kalman.stateZ']]
        
        # if not stop:
        # current_pose_list =np.append(current_pose_list,[[self.pos[0],self.pos[1],self.pos[2],0]], axis=0)
        # setpoint_list = np.append(setpoint_list,[current_setpoint], axis=0) 
        # print(current_setpoint)
        
        
        
    # def _vel_log_data(self, timestamp,data, logconf):
    #     self.vel = np.r_[data[]]
    def _att_log_data(self, timestamp,data, logconf):
        self.attq = np.r_[data['kalman.q0'],
                         data['kalman.q1'],
                         data['kalman.q2'],
                         data['kalman.q3']]
        # self.R = trans.quarternion_matrix(self.attq)[:3, :3]


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

def simple_sequence(cf):
    with SyncCrazyflie(uri, cf=cf) as scf:
        with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_PID) as pc:
            pc.forward(0.5)
            pc.left(0.5)
            pc.back(0.5)
            pc.go_to(0.0, 0.0, 0.5)
def motion_commander_test(cf):
    with SyncCrazyflie(uri, cf=cf) as scf:
            # We take off when the commander is created
            with MotionCommander(scf) as mc:   
                time.sleep(1)

                # There is a set of functions that move a specific distance
                # We can move in all directions
                mc.forward(0.8)
                mc.back(0.8)
                time.sleep(1)

                mc.up(0.5)
                mc.down(0.5)
                time.sleep(1)

                # We can also set the velocity
                mc.right(0.5, velocity=0.8)
                time.sleep(1)
                mc.left(0.5, velocity=0.4)
                time.sleep(1)

                # We can do circles or parts of circles
                mc.circle_right(0.5, velocity=0.5, angle_degrees=180)

                # Or turn
                mc.turn_left(90)
                time.sleep(1)

                # We can move along a line in 3D space
                mc.move_distance(-1, 0.0, 0.5, velocity=0.6)
                time.sleep(1)

                # There is also a set of functions that start a motion. The
                # Crazyflie will keep on going until it gets a new command.

                mc.start_left(velocity=0.5)
                # The motion is started and we can do other stuff, printing for
                # instance
                for _ in range(5):
                    print('Doing other work')
                    time.sleep(0.2)

                # And we can stop
                mc.stop()

                # We land when the MotionCommander goes out of scope
def send_position_test(cf, sequence):
    global current_setpoint
    global setpoint_list
    global stop
    global drone_connected
    print("Waiting for drone to connect")
    while drone_connected == False:
        ...
    print("Drone connected: position command starting")
    for posi in sequence:
        print('Setting position {}'.format(posi))
        for i in range(20):
            for i in range(4):
                current_setpoint[i] = posi[i]
            # print(current_setpoint)
            cf.commander.send_position_setpoint(posi[0],posi[1],posi[2],posi[3])
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    time.sleep(0.1)
    # print(len(setpoint_list))
    # print(len(setpoint_list[0]))
    # stpt_array = np.array(setpoint_list)
    # print(setpoint_list)
    stop = True
    #plotting data 
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')

    ax.plot(setpoint_list[:,0],setpoint_list[:,1],setpoint_list[:,2])
    ax.plot(current_pose_list[:,0],current_pose_list[:,1],current_pose_list[:,2])


    plt.show()
    print("Savin data to file")
    np.savetxt("setpoint.csv",setpoint_list, delimiter="," )
    np.savetxt("sensordata.csv",current_pose_list, delimiter="," )

    # print(current_pose_list)

    


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    cf=Crazyflie(rw_cache='./cache')
    le = ParameterLogging(uri, cf)
    # simple_sequence(cf)
    # motion_commander_test(cf)
    cf.open_link(uri)
    send_position_test(cf, sequence)

 

