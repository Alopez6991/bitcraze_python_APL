#!/usr/bin/env python3
import csv
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from queue import Queue, Empty
from threading import Event, Thread

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


@dataclass(frozen=True)
class Drone:
    name: str
    uri: str


DRONES = [
    Drone("cf0", "radio://0/80/2M/E7E7E7E700"),
    Drone("cf1", "radio://0/80/2M/E7E7E7E701"),
    Drone("cf2", "radio://0/80/2M/E7E7E7E702"),
]

LOG_PERIOD_MS = 200  # start safe
FLUSH_WINDOW_S = 0.030  # 30 ms window to merge split packets into one row

UINT16_VARS = [
    "ranging.distance0", "ranging.distance1", "ranging.distance2",
    "ranging.inD01", "ranging.inD02", "ranging.inD10",
    "ranging.inD12", "ranging.inD20", "ranging.inD21",
]

FLOAT_VARS = [
    "ranging.height0", "ranging.height1", "ranging.height2",
    "ranging.yawR0", "ranging.yawR1", "ranging.yawR2",
    "ranging.vx0", "ranging.vx1", "ranging.vx2",
    "ranging.vy0", "ranging.vy1", "ranging.vy2",
    "ranging.vz0", "ranging.vz1", "ranging.vz2",
    "relLoc.x0", "relLoc.x1", "relLoc.x2",
    "relLoc.y0", "relLoc.y1", "relLoc.y2",
    "relLoc.z0", "relLoc.z1", "relLoc.z2",
    "relLoc.psi0", "relLoc.psi1", "relLoc.psi2",
    "kalman.statePX", "kalman.statePY", "kalman.statePZ",
]

ALL_VARS = UINT16_VARS + FLOAT_VARS


def chunk(lst, n):
    for i in range(0, len(lst), n):
        yield lst[i:i+n]


def connect_and_log(drone: Drone, out_q: Queue, stop_evt: Event) -> None:
    """
    Send packets to writer as:
      ("update", t_wall, drone_name, {var: value, ...})
    """
    with SyncCrazyflie(drone.uri) as scf:
        cf = scf.cf
        logconfs = []

        lc_u16 = LogConfig(name=f"u16_{drone.name}", period_in_ms=LOG_PERIOD_MS)
        for v in UINT16_VARS:
            lc_u16.add_variable(v, "uint16_t")
        logconfs.append(lc_u16)

        # Split floats to avoid payload limits
        for k, vars_chunk in enumerate(chunk(FLOAT_VARS, 5)):
            lc_f = LogConfig(name=f"f{k}_{drone.name}", period_in_ms=LOG_PERIOD_MS)
            for v in vars_chunk:
                lc_f.add_variable(v, "float")
            logconfs.append(lc_f)

        def on_log(ts, data, logconf):
            t_wall = time.time()
            out_q.put(("update", t_wall, drone.name, dict(data)))

        for lc in logconfs:
            cf.log.add_config(lc)
            lc.data_received_cb.add_callback(on_log)
            lc.start()

        try:
            while not stop_evt.is_set():
                time.sleep(0.05)
        finally:
            for lc in logconfs:
                try:
                    lc.stop()
                except Exception:
                    pass


def writer_thread(csv_path: Path, out_q: Queue, stop_evt: Event) -> None:
    """
    Builds a wide row per (drone, frame_start_time).
    Column naming: cfN.<var> like cf1.ranging.vx0
    """
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    # Pre build header
    header = ["t_wall_s", "drone"] + [f"{d.name}.{v}" for d in DRONES for v in ALL_VARS]

    # Per drone assembly buffer
    frames = {}
    # frames[drone] = {
    #   "t0": frame_start_time,
    #   "row": { "cf1.ranging.vx0": value, ... }
    # }

    def flush(drone_name: str):
        fr = frames.get(drone_name)
        if not fr:
            return None

        t0 = fr["t0"]
        rowmap = fr["row"]

        # Build full row with blanks for missing values
        row = [""] * len(header)
        row[0] = f"{t0:.6f}"
        row[1] = drone_name

        # Fill only columns for this drone
        for k, v in rowmap.items():
            try:
                idx = header_index[k]
            except KeyError:
                continue
            row[idx] = f"{v:.10g}"

        frames.pop(drone_name, None)
        return row

    header_index = {name: i for i, name in enumerate(header)}

    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)

        last_flush_check = time.time()

        while not stop_evt.is_set() or not out_q.empty():
            try:
                msg = out_q.get(timeout=0.1)
            except Empty:
                msg = None

            now = time.time()

            # periodic flush for old frames
            if now - last_flush_check > 0.05:
                last_flush_check = now
                for dn in list(frames.keys()):
                    if now - frames[dn]["t0"] >= FLUSH_WINDOW_S:
                        row = flush(dn)
                        if row is not None:
                            w.writerow(row)

            if msg is None:
                continue

            kind, t_wall, drone_name, data = msg

            if drone_name not in frames:
                frames[drone_name] = {"t0": t_wall, "row": {}}

            fr = frames[drone_name]

            # If this update is outside the current merge window, flush previous
            if t_wall - fr["t0"] >= FLUSH_WINDOW_S:
                row = flush(drone_name)
                if row is not None:
                    w.writerow(row)
                frames[drone_name] = {"t0": t_wall, "row": {}}
                fr = frames[drone_name]

            # Add data into the frame row
            for var, val in data.items():
                col = f"{drone_name}.{var}"
                fr["row"][col] = float(val)

            # If we already have all vars for this drone, flush immediately
            need = len(ALL_VARS)
            have = sum(1 for v in ALL_VARS if f"{drone_name}.{v}" in fr["row"])
            if have >= need:
                row = flush(drone_name)
                if row is not None:
                    w.writerow(row)

        # Final flush on exit
        for dn in list(frames.keys()):
            row = flush(dn)
            if row is not None:
                w.writerow(row)


def main() -> int:
    cflib.crtp.init_drivers(enable_debug_driver=False)

    out_q: Queue = Queue()
    stop_evt = Event()

    def stop(*_):
        stop_evt.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = Path.home() / "cf_logs" / f"ranging_3cf_wide_{stamp}.csv"

    wt = Thread(target=writer_thread, args=(csv_path, out_q, stop_evt), daemon=True)
    wt.start()

    threads = []
    for d in DRONES:
        t = Thread(target=connect_and_log, args=(d, out_q, stop_evt), daemon=True)
        t.start()
        threads.append(t)

    print("Logging wide ranging snapshots. Ctrl+C to stop.")
    print(f"Saving to: {csv_path}")

    try:
        while not stop_evt.is_set():
            time.sleep(0.2)
    finally:
        stop_evt.set()
        for t in threads:
            t.join(timeout=2.0)
        wt.join(timeout=2.0)

    print(f"Done. Saved: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())