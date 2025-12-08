from threading import Thread

from cf_utils import *

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

uris_to_kill = [
    'radio://0/80/2M/E7E7E7E708'
    # 'radio://0/80/2M/E7E7E7E709'
]

if __name__ == '__main__':
    print('initializing drivers')
    cflib.crtp.init_drivers()

    for uri in uris_to_kill:
        cf_uri = uri_helper.uri_from_env(default=uri)
        print(f"Try disarming {uri}")
        try:
            with SyncCrazyflie(cf_uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                cf = scf.cf
                print(f"Disarming {uri}")
                cf.platform.send_arming_request(False)
                time.sleep(0.2)
        except Exception as ex:
            print(ex)
