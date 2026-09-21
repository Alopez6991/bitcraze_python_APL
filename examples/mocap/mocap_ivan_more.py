# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2023 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Example of how to connect to a motion capture system and feed the position to a
Crazyflie, using the motioncapture library. The motioncapture library supports all major mocap systems and provides
a generalized API regardless of system type.
The script uses the high level commander to upload a trajectory to fly a figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
mocap setting matching your system.
"""
import random
import time
from threading import Thread

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# The host name or ip address of the mocap system
host_name = '192.168.209.81'


# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'optitrack'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'flapper_02'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 4.5e-3


# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

snn_control = False

# battery variables
batt_level = 0
batt_state = 0

# time variables
t_start = 0


class ConnectionLostError(Exception):
    pass


class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        print('Connecting to mocap system')
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        print('Connecting to optitrack successful')
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    # print(self.on_pose)
                    if self.on_pose:
                        pos = obj.position

                        # print(f"Position: ({-pos[1]}, {pos[0]}, {pos[2]})")
                        # rotation = {"w": obj.rotation.w, "x": -obj.rotation.y, "y": obj.rotation.x, "z": obj.rotation.z}
                        # rotation = [obj.rotation.w, obj.rotation.y, -obj.rotation.x, obj.rotation.z]
                        # 0 = y, 1 = -x, 2 = z
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])
            # print(3)


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

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

            print('{} {} {}'.
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if send_full_pose:
        cf.extpos.send_extpose(z, x, y, quat.z, quat.x, quat.y, quat.w)
        # cf.extpos.send_extpose(-y, x, z, -quat.y, quat.x, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(z, x, y)
        # print(-y, x, z)
        # cf.extpos.send_extpos(-y, x, z)


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    # time.sleep(1)
    wait_for_position_estimator(cf)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def activate_snn_controller(cf):
    cf.param.set_value('pid_rate.snnEn', '1')


def deactivate_snn_controller(cf):
    cf.param.set_value('pid_rate.snnEn', '0')


def set_snn_I_gain(cf, gain):
    cf.param.set_value('pid_rate.snnIGain', str(gain))


def start_onboard_logging(cf):
    cf.param.set_value('usd.logging', '1')
    print('Onboard logging started.')


def stop_onboard_logging(cf):
    cf.param.set_value('usd.logging', '0')
    print('Onboard logging stopped.')


def get_battery_level(cf):
    log_config = LogConfig(name='Battery', period_in_ms=500)
    log_config.add_variable('pm.vbat', 'float')
    with SyncLogger(cf, log_config) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            return data['pm.vbat']


def get_battery_state(cf):
    log_config = LogConfig(name='State', period_in_ms=500)
    log_config.add_variable('pm.state', 'int8_t')
    with SyncLogger(cf, log_config) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            return data['pm.state']


def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    trajectory_mem.write_data_sync()
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration


def run_sequence(cf):
    global batt_level, batt_state, t_start
    # Starting position
    x = -2
    y = 0
    z = 1
    # yaw = 90
    yaw = 0.0
    cf.platform.send_arming_request(True)
    time.sleep(3.0)
    commander = cf.high_level_commander
    t_start = time.time()
    commander.takeoff(1.0, 2.0)
    time.sleep(6.0)
    # go to starting position
    print('Going to starting position at x=-2')
    commander.go_to(x, y, z, yaw, 1)
    time.sleep(6.0)
    print('Moving to x=2')
    commander.go_to(x + 4, y, z, yaw, 1)
    time.sleep(6.0)

    print('Moving back to x=-2')
    commander.go_to(x, y, z, yaw, 1)
    time.sleep(6.0)
    print('Landing')
    commander.land(0.0, 2.0)
    time.sleep(6.0)
    stop_onboard_logging(cf)
    commander.stop()


def reconnect_and_land():
    start_time = time.time()
    duration = 10
    while time.time() - start_time < duration:
        print('Try to reconnect')
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print('Recovered connection and stopping propellors')
                cf = scf.cf
                cf.high_level_commander.stop()
        except Exception as e:
            print('Connection failed: ', e)
            time.sleep(1)


def connection_failed_link_error(link_uri, msg):
    print(f"Connection to {link_uri} failed: {msg}")
    reconnect_and_land()


def log_batt_callback(timestamp, data, logconf):
    global batt_level, batt_state, t_start
    print(f"[{time.time() - t_start:.2f}s] Batt. level: {data['pm.vbat']:0.2f}V, " +
          f"state: {data['pm.state']}, " +
          f"target: {data['posCtl.targetX']:0.2f}, " +
          f"locSrv: {data['locSrv.x']:0.2f}, " +
          f"stateEstimate: {data['stateEstimate.x']:0.2f}")
    batt_level = data['pm.vbat']
    batt_state = data['pm.state']


def add_logconfig(cf):
    log_config = LogConfig(name='Battery', period_in_ms=2000)
    log_config.add_variable('pm.vbat', 'float')
    log_config.add_variable('pm.state', 'int8_t')
    log_config.add_variable('posCtl.targetX', 'float')
    log_config.add_variable('locSrv.x', 'float')
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.data_received_cb.add_callback(log_batt_callback)
    cf.log.add_config(log_config)
    log_config.start()
    return log_config


def stop_logconfig(logconfig):
    logconfig.stop()
    logconfig.data_received_cb.remove_callback(log_batt_callback)


def console_incoming(console_text):
    print(console_text, end='')


if __name__ == '__main__':
    print('initializing drivers')
    cflib.crtp.init_drivers()

    print('Initializing MocapWrapper')
    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)

    print('Connect to the Crazyflie')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        mission_completed = False

        cf.connection_lost.add_callback(connection_failed_link_error)
        # cf.console.receivedChar.add_callback(console_incoming)

        log_config = add_logconfig(cf)

        # Set up a callback to handle data from the mocap system
        mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])

        # adjust_orientation_sensitivity(cf)
        print('Activating the kalman estimator')
        activate_kalman_estimator(cf)
        reset_estimator(cf)
        start_onboard_logging(cf)
        try:
            run_sequence(cf)
            mission_completed = True
            time.sleep(1.0)
        finally:
            if not mission_completed:
                stop_onboard_logging(cf)
            stop_logconfig(log_config)

    mocap_wrapper.close()
