#!/usr/bin/env python3
"""
Explicit takeoff/land + Flow-deck check (CF 2.1 brushless friendly)

- Arms (CF 2.1 brushless)
- Forces Kalman estimator (needed to use Flow)
- Prints whether a Flow deck is detected
- Explicitly take_off() and land()
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Change this to your Crazyflie URI
URI = 'radio://0/80/2M/E7E7E7E709'


def _get_param(scf, name):
    """Best-effort getter for a parameter; returns None if missing."""
    try:
        return scf.cf.param.get_value(name)
    except Exception:
        return None


def console_cb(msg):
    print(msg, end='')


def _truthy(v):
    return str(v).lower() in ('1', 'true', 'yes', 'on')


def print_flow_status(scf):
    """
    Check common Flow-deck detection flags and print status.
    Also prints which estimator is selected.
    """
    # Different firmware versions expose different param names; try several.
    candidates = ('deck.bcFlow2', 'deck.bcFlow', 'deck.flow', 'deck.flow2', 'deck.pmw3901')
    detected = False
    print('\n=== Sensor/estimator status ===')
    for name in candidates:
        val = _get_param(scf, name)
        if val is not None:
            print(f"{name} = {val}")
            if _truthy(val):
                detected = True

    est = _get_param(scf, 'stabilizer.estimator')
    print(f"stabilizer.estimator = {est}  (2 = Kalman)")
    if detected and est == '2':
        print('Flow deck: DETECTED and will be USED by the Kalman estimator ✅\n')
    elif detected:
        print('Flow deck: DETECTED but Kalman is not selected — set stabilizer.estimator=2 to use it.\n')
    else:
        print('Flow deck: NOT detected — hover will drift without position/vel feedback.\n')
    return detected


def main():
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        scf.cf.console.receivedChar.add_callback(console_cb)

        # Brushless CF2.1 needs arming
        if hasattr(scf.cf.platform, 'send_arming_request'):
            print('Arming (CF 2.1 brushless)...')
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)

        # Ensure Kalman is selected so Flow can be used
        scf.cf.param.set_value('stabilizer.estimator', '2')
        time.sleep(0.1)

        # (Optional) fresh start for the Kalman filter
        try:
            scf.cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            scf.cf.param.set_value('kalman.resetEstimation', '0')
        except Exception:
            pass  # param may not exist on older firmwares

        _ = print_flow_status(scf)

        mc = None
        try:
            # Create MotionCommander (normally auto-takes off to default_height)
            mc = MotionCommander(scf, default_height=0.3)

            # Explicit takeoff to a target height (safe & clear)
            print('Taking off to 1.5 m...')
            mc.take_off(height=1.5, velocity=0.5)
            time.sleep(2.0)

            # print("Forward 1.5 m")
            # mc.forward(1.5, velocity=0.5)
            # time.sleep(2.0)

            # print("right 1.5 m")
            # mc.right(1.5, velocity=0.5)
            # time.sleep(2.0)

            print('forward 5.0 m')
            mc.forward(3.5, velocity=1.5)
            time.sleep(2.0)

            print('Hover 1.0 s')
            time.sleep(1.0)

            # Stop before landing to avoid 'on the ground' exception
            print('Stopping (hover hold) ...')
            try:
                mc.stop()
            except Exception:
                pass
            time.sleep(0.2)

            print('Landing...')
            mc.land(velocity=0.3)
            time.sleep(1.5)  # allow time to settle

        finally:
            # Safety: stop any motion & disarm
            if mc is not None:
                mc.stop()
            if hasattr(scf.cf.platform, 'send_arming_request'):
                print('Disarming...')
                scf.cf.platform.send_arming_request(False)

    print('Done.')


if __name__ == '__main__':
    main()
