# -*- coding: utf-8 -*-
import os
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


def _read_yaml(yaml_path):
    name = None
    duration = 5.0
    p = os.path.abspath(yaml_path)
    if os.path.isfile(p):
        try:
            with open(p, 'r', encoding='utf-8') as f:
                for raw in f:
                    s = raw.strip()
                    if not s or s.startswith('#') or ':' not in s:
                        continue
                    k, v = s.split(':', 1)
                    k = k.strip()
                    v = v.strip()
                    if k in ('usd_name', 'usd.name', 'session_name') and not name:
                        name = v[:12]
                    elif k in ('duration_s', 'usd_logging_seconds', 'log_seconds'):
                        try:
                            duration = max(0.1, float(v))
                        except ValueError:
                            pass
        except Exception as e:
            print(f"YAML read error ({p}): {e}")
    return name, duration


def _set_usd_name_bytes(cf, name12):
    # Write ASCII bytes to usd.name0..usd.name11, and terminate with 0 if shorter
    for i in range(12):
        b = ord(name12[i]) if (name12 and i < len(name12)) else 0
        try:
            cf.param.set_value(f'usd.name{i}', str(b))
        except Exception as e:
            print(f"[usd] failed to set name{i}: {e}")


def main():
    # CLI: --filename=param.yaml
    yaml_path = None
    for arg in sys.argv[1:]:
        if arg.startswith('--filename='):
            yaml_path = arg.split('=', 1)[1]
        elif arg in ('--help', '-h'):
            print('Usage: python3 file_name_test.py --filename=param.yaml')
            return
    if not yaml_path:
        print('No YAML provided. Use --filename=param.yaml (current dir)')
        return

    name, duration = _read_yaml(yaml_path)

    cflib.crtp.init_drivers()
    uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        try:
            can_log = int(cf.param.get_value('usd.canLog'))
            has_sd = int(cf.param.get_value('deck.bcUSD'))
            print(f"[usd] canLog={can_log} bcUSD={has_sd}")
        except Exception:
            pass

        if name:
            _set_usd_name_bytes(cf, name)
            print(f"[usd] session name set to '{name}'")

        try:
            cf.param.set_value('usd.logging', '1')
            print('[usd] logging START')
        except Exception as e:
            print(f"[usd] failed to start logging: {e}")
            return

        time.sleep(duration)

        try:
            cf.param.set_value('usd.logging', '0')
            print('[usd] logging STOP')
        except Exception as e:
            print(f"[usd] failed to stop logging: {e}")


if __name__ == '__main__':
    main()
