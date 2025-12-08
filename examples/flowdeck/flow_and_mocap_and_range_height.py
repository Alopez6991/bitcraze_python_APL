#!/usr/bin/env python3
"""
Merged Flow+Mocap script
- Flight & commands use Flow deck (Kalman estimator = 2)
- Mocap is LOG-ONLY to CSV (does NOT influence estimator)
- CF state + Multi-ranger also logged to CSV on the host (one file)
- Terrain spike monitor prints alerts to terminal (no file logging)

Outputs (under logs/log_YY_MM_DD_02/):
  mocap_<epoch>.csv  -> t, x, y, z, qw, qx, qy, qz
  cf_<epoch>.csv     -> t, pm.vbat, pm.state, stateEstimate.{x,y,z}, stabilizer.yaw, posCtl.targetX, locSrv.x,
                        range.front, range.back, range.left, range.right, range.up
"""
import csv
import logging
import time
from pathlib import Path
from threading import Lock
from threading import Thread

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# -----------------------------
# User params
# -----------------------------
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E709')

# Mocap settings
MOCAP_HOST = '192.168.209.81'
MOCAP_TYPE = 'optitrack'  # 'vicon'|'optitrack'|'optitrack_closed_source'|'qualisys'|'nokov'|'vrpn'|'motionanalysis'
RIGID_BODY = 'buzz'

# Flight pattern
TAKEOFF_H = 1.0
TAKEOFF_VEL = 0.5
FWD_METERS = 7.0
FWD_VEL = 0.5
HOVER_S = 2.0

# Range unit hint (you said it's in mm)
RANGE_UNITS = 'mm'  # 'mm' or 'm'

# -----------------------------
# Mocap thread (observe-only)
# -----------------------------


class MocapWrapper(Thread):
    def __init__(self, body_name, host, sys_type, on_pose):
        super().__init__(daemon=True)
        self.body_name = body_name
        self.host = host
        self.sys_type = sys_type
        self.on_pose = on_pose  # callable([x,y,z, quat])
        self._stay_open = True

    def close(self):
        self._stay_open = False

    def run(self):
        print('Connecting to mocap system...')
        mc = motioncapture.connect(self.sys_type, {'hostname': self.host})
        print(f"Connected to mocap ({self.sys_type}) at {self.host}")
        while self._stay_open:
            mc.waitForNextFrame()
            rb = mc.rigidBodies.get(self.body_name)
            if rb is not None and self.on_pose:
                pos = rb.position
                self.on_pose([pos[0], pos[1], pos[2], rb.rotation])


# -----------------------------
# Flow deck status (from flow_test)
# -----------------------------
def _get_param(scf, name):
    try:
        return scf.cf.param.get_value(name)
    except Exception:
        return None


def _truthy(v):
    return str(v).lower() in ('1', 'true', 'yes', 'on')


def print_flow_status(scf):
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


# -----------------------------
# Main
# -----------------------------
def main():
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Prepare logging dirs/files (dated folder)
    t0 = time.time()
    date_str = time.strftime('%y_%m_%d')
    out_root = Path('logs') / f"log_{date_str}_02"
    out_root.mkdir(parents=True, exist_ok=True)

    mocap_path = out_root / f"mocap_{int(t0)}.csv"
    cf_path = out_root / f"cf_{int(t0)}.csv"

    mocap_f = open(mocap_path, 'w', newline='')
    cf_f = open(cf_path, 'w', newline='')
    mocap_w = csv.writer(mocap_f)
    cf_w = csv.writer(cf_f)

    mocap_w.writerow(['t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
    cf_w.writerow([
        't',
        'pm.vbat', 'pm.state',
        'stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
        'stabilizer.yaw', 'posCtl.targetX', 'locSrv.x',
        'range.front', 'range.back', 'range.left', 'range.right', 'range.up'
    ])

    # Protect writers (callbacks run in other threads)
    write_lock = Lock()

    # --- consolidated CF row writer (fills NaNs when keys missing)
    def _write_cf_row(data: dict):
        with write_lock:
            cf_w.writerow([
                time.time() - t0,
                data.get('pm.vbat', float('nan')),
                data.get('pm.state', float('nan')),
                data.get('stateEstimate.x', float('nan')),
                data.get('stateEstimate.y', float('nan')),
                data.get('stateEstimate.z', float('nan')),
                data.get('stabilizer.yaw', float('nan')),
                data.get('posCtl.targetX', float('nan')),
                data.get('locSrv.x', float('nan')),
                data.get('range.front', float('nan')),
                data.get('range.back', float('nan')),
                data.get('range.left', float('nan')),
                data.get('range.right', float('nan')),
                data.get('range.up', float('nan')),
            ])
            # cf_f.flush()  # optional safety flush

    # Mocap callback: LOG ONLY
    def on_mocap_pose(pose):
        x, y, z, q = pose
        with write_lock:
            mocap_w.writerow([time.time() - t0, x, y, z, q.w, q.x, q.y, q.z])
            # mocap_f.flush()

    # Start mocap thread
    mocap_thread = MocapWrapper(RIGID_BODY, MOCAP_HOST, MOCAP_TYPE, on_mocap_pose)
    mocap_thread.start()

    mc = None
    log1 = log2 = log3 = log4 = None  # include log4 for monitor
    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            # Brushless arming (does not spin props)
            if hasattr(cf.platform, 'send_arming_request'):
                print('Arming (CF 2.1 brushless)...')
                cf.platform.send_arming_request(True)
                time.sleep(1.0)

            # Ensure Kalman (Flow needs estimator=2)
            cf.param.set_value('stabilizer.estimator', '2')
            time.sleep(0.1)

            # Fresh estimator (best-effort)
            try:
                cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                cf.param.set_value('kalman.resetEstimation', '0')
            except Exception:
                pass

            _ = print_flow_status(scf)

            # ---- Host-side CF loggers (split + safe) ----
            def safe_add(logconf, var, vtype):
                try:
                    logconf.add_variable(var, vtype)
                    return True
                except Exception:
                    print(f"[log] Skipping missing/invalid var: {var}")
                    return False

            # Log block 1: state estimate & yaw @100 ms
            log1 = LogConfig(name='hostlog1', period_in_ms=100)
            for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'stabilizer.yaw'):
                safe_add(log1, v, 'float')

            # Log block 2: battery & extras @200 ms
            log2 = LogConfig(name='hostlog2', period_in_ms=200)
            for v, t in (('pm.vbat', 'float'), ('pm.state', 'int8_t'),
                         ('posCtl.targetX', 'float'), ('locSrv.x', 'float')):
                safe_add(log2, v, t)

            # Log block 3: Multi-ranger @100 ms
            log3 = LogConfig(name='multiranger', period_in_ms=100)
            for v in ['range.front', 'range.back', 'range.left', 'range.right', 'range.up']:
                safe_add(log3, v, 'float')

            # Callbacks: send *all* blocks through the same writer
            def _cf_cb(timestamp, data, logconf):  # state/battery
                _write_cf_row(data)

            def _rng_cb(timestamp, data, logconf):  # ranger
                _write_cf_row(data)

            # Start logs 1..3
            for lc in (log1, log2, log3):
                try:
                    cf.log.add_config(lc)
                    if lc is log3:
                        lc.data_received_cb.add_callback(_rng_cb)
                    else:
                        lc.data_received_cb.add_callback(_cf_cb)
                    lc.start()
                except Exception as e:
                    print(f"[log] Could not start {lc.name}: {e}")

            # ---- Terrain spike monitor (prints only, no file logging) ----
            log4 = LogConfig(name='terrain_monitor', period_in_ms=50)

            # Candidate variable names for downward range (Flow deck)
            down_candidates = [
                'range.zrange',       # Flow v2 ZRanger (most common)
                'range.zDistance',
                'range.distance',
                'range.down',
                'range.altitude',     # some forks
            ]
            # Candidate variable names for vertical acceleration
            accz_candidates = [
                'acc.z',
                'stabilizer.accelZ',
                'imu.accZ',
            ]

            down_var = None
            accz_var = None

            for v in down_candidates:
                try:
                    log4.add_variable(v, 'float')
                    down_var = v
                    break
                except Exception:
                    pass

            for v in accz_candidates:
                try:
                    log4.add_variable(v, 'float')
                    accz_var = v
                    break
                except Exception:
                    pass

            if down_var is None and accz_var is None:
                print('[monitor] No down-range or accZ variables found — skipping terrain monitor')
                log4 = None
            else:
                from collections import deque
                r_hist = deque(maxlen=6)
                t_hist = deque(maxlen=6)

                # Thresholds tuned for RANGE_UNITS == 'mm'
                if RANGE_UNITS == 'mm':
                    STEP_THRESH = 50.0     # 50 mm step
                    RATE_THRESH = 500.0    # 500 mm/s rate
                    scale_to_m = 1.0 / 1000.0
                else:
                    STEP_THRESH = 0.05     # 5 cm
                    RATE_THRESH = 0.50     # 0.5 m/s
                    scale_to_m = 1.0

                ACCZ_THRESH = 3.0  # m/s^2 deviation from g

                def _monitor_cb(ts, data, logconf):
                    now = time.time() - t0
                    r = data.get(down_var) if down_var else None
                    az = data.get(accz_var) if accz_var else None

                    # --- Downward range spike check (signed) ---
                    if r is not None:
                        r_hist.append(float(r))
                        t_hist.append(now)
                        if len(r_hist) >= 2:
                            r_now = r_hist[-1]
                            r_prev = r_hist[-2]
                            dt = max(t_hist[-1] - t_hist[-2], 1e-3)
                            delta = r_now - r_prev               # signed, in raw units
                            rate = delta / dt                   # raw units per second

                            # Check magnitude in raw units (mm or m) against tuned thresholds
                            if delta <= -STEP_THRESH or abs(rate) >= RATE_THRESH:
                                # Range dropped → terrain rose under you
                                print(
                                    f"[⬆️ terrain rise] Δr={delta*scale_to_m:.3f} m, rate={(rate*scale_to_m):.3f} m/s")
                            elif delta >= STEP_THRESH or abs(rate) >= RATE_THRESH:
                                # Range increased → you left the object
                                print(
                                    f"[⬇️ terrain drop] Δr={delta*scale_to_m:.3f} m, rate={(rate*scale_to_m):.3f} m/s")

                    # --- Accel Z spike check ---
                    if az is not None:
                        az = float(az)
                        # If |az| < 5 it is likely in g, convert to m/s^2
                        az_ms2 = az * 9.81 if abs(az) < 5.0 else az
                        dev = abs(az_ms2 - 9.81)
                        if dev > ACCZ_THRESH:
                            print(f"[⚠️ accel-Z spike] |az - g| = {dev:.2f} m/s²")

                try:
                    cf.log.add_config(log4)
                    log4.data_received_cb.add_callback(_monitor_cb)
                    log4.start()
                    print(f"[monitor] Active. down_var={down_var}, accz_var={accz_var}, units={RANGE_UNITS}")
                except Exception as e:
                    print(f"[monitor] Could not start: {e}")
                    log4 = None

            # ---- Flight using Flow deck (no mocap in estimator) ----
            time.sleep(8.0)  # pre-flight logging window
            mc = MotionCommander(scf, default_height=0.3)

            print(f"Taking off to {TAKEOFF_H} m...")
            mc.take_off(height=TAKEOFF_H, velocity=TAKEOFF_VEL)
            time.sleep(1.5)

            print(f"Hover {HOVER_S:.1f} s")
            time.sleep(HOVER_S)

            print(f"Forward {FWD_METERS} m @ {FWD_VEL} m/s")
            mc.forward(FWD_METERS, velocity=FWD_VEL)
            time.sleep(2.0)

            print(f"Hover {HOVER_S:.1f} s")
            time.sleep(HOVER_S)

            print('Stopping (hover hold)...')
            try:
                mc.stop()
            except Exception:
                pass
            time.sleep(0.2)

            print('Landing...')
            mc.land(velocity=0.3)
            time.sleep(1.5)

    finally:
        # Cleanup
        for lc in (log1, log2, log3, log4):
            try:
                if lc is not None:
                    lc.stop()
            except Exception:
                pass

        try:
            if mc is not None:
                mc.stop()
        except Exception:
            pass

        # Disarm if present
        try:
            with SyncCrazyflie(URI) as scf2:
                if hasattr(scf2.cf.platform, 'send_arming_request'):
                    print('Disarming...')
                    scf2.cf.platform.send_arming_request(False)
        except Exception:
            pass

        # Stop mocap thread
        try:
            mocap_thread.close()
        except Exception:
            pass
        try:
            mocap_thread.join(timeout=1.0)
        except Exception:
            pass

        # Close CSVs
        mocap_f.close()
        cf_f.close()

        print('Logs written to:')
        print(f"  {mocap_path}")
        print(f"  {cf_path}")
        print('Done.')


if __name__ == '__main__':
    main()
