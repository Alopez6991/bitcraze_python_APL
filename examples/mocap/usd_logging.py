# -*- coding: utf-8 -*-
"""
Minimal script to start/stop onboard USD logging on a Crazyflie.
Logging starts on connect and stops cleanly on Ctrl+C or exit.
"""
import signal
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')


def start_onboard_logging(cf):
    cf.param.set_value('usd.logging', '1')
    print('Onboard logging started.')


def stop_onboard_logging(cf):
    cf.param.set_value('usd.logging', '0')
    print('Onboard logging stopped.')


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        start_onboard_logging(cf)

        # Register a clean shutdown on Ctrl+C
        def shutdown(sig, frame):
            stop_onboard_logging(cf)

        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)

        print('Logging... Press Ctrl+C to stop.')
        signal.pause()  # Block until signal received
