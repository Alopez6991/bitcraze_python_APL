import time

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
ORIENTATION_STD_DEV = 4.5e-3

SEND_FULL_POSE = True


def add_logconfig(cf, variables, callback, freq=1000):
    log_config = LogConfig(name='Logging', period_in_ms=freq)
    for v in variables:
        log_config.add_variable(v['name'], v['type'])
    log_config.data_received_cb.add_callback(callback)
    cf.log.add_config(log_config)
    log_config.start()
    return log_config

def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', ORIENTATION_STD_DEV)

def activate_snn_controller(cf):
    cf.param.set_value('pid_rate.snnEn', '1')
    time.sleep(0.1)

def deactivate_snn_controller(cf):
    cf.param.set_value('pid_rate.snnEn', '0')

def set_snn_I_gain(cf, gain):
    cf.param.set_value('pid_rate.snnIGain', str(gain))
    time.sleep(0.1)

def set_snn_type(cf):
    cf.param.set_value('pid_rate.snnType', '4')

def start_onboard_logging(cf):
    cf.param.set_value("usd.logging", "1")
    time.sleep(0.05)

def stop_onboard_logging(cf):
    cf.param.set_value("usd.logging", "0")
    time.sleep(0.05)


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

def console_incoming(console_text):
    print(console_text, end='')

def connection_failed_link_error(link_uri, msg):
    print(f"Connection to {link_uri} failed: {msg}")
    reconnect_and_land()


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    log_config.add_variable('kalman.stateX', 'float')
    log_config.add_variable('kalman.stateY', 'float')
    log_config.add_variable('kalman.stateZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.05

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

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))
            print(f"x: {data['kalman.stateX']}, y: {data['kalman.stateY']}, z: {data['kalman.stateZ']}")

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.1)

def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if SEND_FULL_POSE:
        cf.extpos.send_extpose(z, x, y, quat.z, quat.x, quat.y, quat.w)
        # cf.extpos.send_extpose(-y, x, z, -quat.y, quat.x, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(z, x, y)
        # print(-y, x, z)
        # cf.extpos.send_extpos(-y, x, z)

def reconnect_and_land(uri):
    start_time = time.time()
    duration = 10
    while time.time() - start_time < duration:
        print("Try to reconnect")
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print("Recovered connection and stopping propellors")
                cf = scf.cf
                cf.high_level_commander.stop()
        except Exception as e:
            print("Connection failed: ", e)
            time.sleep(1)