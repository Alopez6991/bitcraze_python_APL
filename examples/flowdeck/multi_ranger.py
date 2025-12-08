#!/usr/bin/env python3
"""
Multi-ranger live readout (no flying)

- Arms (CF 2.1 brushless) for permission, but does NOT spin motors or fly
- Subscribes to range.{front,back,left,right,up}
- Prints values at a steady rate
- Gracefully handles missing variables if the deck/sensor isn't present
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Change this to your Crazyflie URI
URI = 'radio://0/80/2M/E7E7E7E709'

# How often to print (Hz) and for how long (seconds)
PRINT_HZ = 10
DURATION_S = 200


def main():
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        cf = scf.cf

        # # Brushless CF 2.1 needs arming to accept some commands; this does NOT spin motors.
        # if hasattr(cf.platform, 'send_arming_request'):
        #     print("Arming (CF 2.1 brushless)...")
        #     cf.platform.send_arming_request(True)
        #     time.sleep(0.5)

        # Prepare a log block for the Multi-ranger variables
        lg = LogConfig(name='multiranger', period_in_ms=int(1000 / PRINT_HZ))

        # Some firmwares expose all five; add best-effort
        candidates = [
            ('range.front', 'float'),
            ('range.back', 'float'),
            ('range.left', 'float'),
            ('range.right', 'float'),
            ('range.up', 'float'),
        ]

        added = []
        for name, vtype in candidates:
            try:
                lg.add_variable(name, vtype)
                added.append(name)
            except Exception:
                # Variable not available in this firmware/deck combo
                pass

        if not added:
            print('No Multi-ranger variables available (is the deck attached and detected?).')
            # Disarm if we armed
            # if hasattr(cf.platform, 'send_arming_request'):
            #     print("Disarming...")
            #     cf.platform.send_arming_request(False)
            return

        latest = {k: float('nan') for k in ['range.front', 'range.back', 'range.left', 'range.right', 'range.up']}

        def _cb(timestamp, data, logconf):
            # Update whatever we received this tick
            for k in latest.keys():
                if k in data:
                    latest[k] = float(data[k])

        # Start logging
        try:
            cf.log.add_config(lg)
            lg.data_received_cb.add_callback(_cb)
            lg.start()
        except Exception as e:
            print(f"Could not start log config: {e}")
            # if hasattr(cf.platform, 'send_arming_request'):
            #     print("Disarming...")
            #     cf.platform.send_arming_request(False)
            return

        print('\nReading Multi-ranger... (Ctrl+C to stop)')
        print('  Distances are in meters; NaN means no return / out of range.\n')

        t0 = time.time()
        try:
            while time.time() - t0 < DURATION_S:
                # Pretty print in a fixed layout
                f = latest['range.front']
                b = latest['range.back']
                l = latest['range.left']
                r = latest['range.right']
                u = latest['range.up']

                # Format helper
                def fmt(v):
                    return f"{v:5.2f}" if (v == v) else '  NaN'

                line = (
                    f"front: {fmt(f)}   back: {fmt(b)}   "
                    f"left: {fmt(l)}   right: {fmt(r)}   up: {fmt(u)}"
                )
                print(line)
                time.sleep(1.0 / PRINT_HZ)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                lg.stop()
            except Exception:
                pass

            if hasattr(cf.platform, 'send_arming_request'):
                print('Disarming...')
                # cf.platform.send_arming_request(False)

    print('Done.')


if __name__ == '__main__':
    main()
