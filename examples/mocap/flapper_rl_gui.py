# -*- coding: utf-8 -*-
"""
Mocap streaming + state GUI for the Flapper running the RL controller app.

- Streams extpose from a mocap system to the flapper at ...E700.
- Exposes three buttons that switch the firmware's `rlapp.targetState`:
    0 = IDLE        (land + cut motors)
    1 = HOVERING    (PID hover at targetAlt, calibrating servo zero)
    2 = RL_CONTROL  (NN motor override active)
- Shows live battery, current firmware state, target gate, and NN outputs.

Edit the constants at the top to match your mocap host / rigid body name.
"""
import time
import tkinter as tk
from threading import Thread

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# ---- User configuration ----------------------------------------------------
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

host_name = '192.168.209.81'
mocap_system_type = 'optitrack'   # vicon, optitrack, qualisys, vrpn, ...
rigid_body_name = 'flapper_02'

send_full_pose = True
orientation_std_dev = 0.06        # std dev pushed to locSrv.extQuatStdDev

STATE_IDLE = 0
STATE_HOVERING = 1
STATE_RL_CONTROL = 2

STATE_NAMES = {
    STATE_IDLE: 'IDLE',
    STATE_HOVERING: 'HOVERING',
    STATE_RL_CONTROL: 'RL_CONTROL',
}


# ---- Mocap thread ----------------------------------------------------------
class MocapWrapper(Thread):
    def __init__(self, body_name):
        super().__init__(daemon=True)
        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        print(f'Connecting to mocap ({mocap_system_type}@{host_name})')
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        print('Mocap connected')
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name and self.on_pose:
                    pos = obj.position
                    self.on_pose([pos[0], pos[1], pos[2], obj.rotation])


def send_extpose_quat(cf, x, y, z, quat):
    if send_full_pose:
        cf.extpos.send_extpose(z, x, y, quat.z, quat.x, quat.y, quat.w)
    else:
        cf.extpos.send_extpos(z, x, y)


# ---- Estimator setup -------------------------------------------------------
def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(1.5)


# ---- GUI -------------------------------------------------------------------
class RLControlGui:
    def __init__(self, root, cf):
        self.cf = cf
        self.root = root
        self.live = {
            'state': tk.StringVar(value='--'),
            'tgtGate': tk.StringVar(value='--'),
            'vbat': tk.StringVar(value='-- V'),
            'nn': tk.StringVar(value='--'),
            'pos': tk.StringVar(value='--'),
        }

        root.title('Flapper RL controller')
        root.geometry('420x500')

        # Kill button ---------------------------------------------------------
        # Big, always-visible emergency stop. Also bound to Space and 'k'.
        kill_frame = tk.Frame(root, padx=8, pady=6)
        kill_frame.pack(fill='x')
        tk.Button(kill_frame, text='KILL  (Space / K)',
                  bg='#cc0000', fg='white',
                  activebackground='#990000', activeforeground='white',
                  font=('TkDefaultFont', 14, 'bold'),
                  height=2, command=self.kill).pack(fill='x')
        root.bind('<space>', lambda _e: self.kill())
        root.bind('<KeyPress-k>', lambda _e: self.kill())
        root.bind('<KeyPress-K>', lambda _e: self.kill())

        # State buttons -------------------------------------------------------
        btn_frame = tk.LabelFrame(root, text='Target state', padx=8, pady=8)
        btn_frame.pack(fill='x', padx=8, pady=6)

        tk.Button(btn_frame, text='IDLE (land)', width=14, bg='#f4cccc',
                  command=lambda: self.set_state(STATE_IDLE)).pack(side='left', padx=4)
        tk.Button(btn_frame, text='HOVER', width=14, bg='#fff2cc',
                  command=lambda: self.set_state(STATE_HOVERING)).pack(side='left', padx=4)
        tk.Button(btn_frame, text='RL CONTROL', width=14, bg='#d9ead3',
                  command=lambda: self.set_state(STATE_RL_CONTROL)).pack(side='left', padx=4)

        # Param entries -------------------------------------------------------
        param_frame = tk.LabelFrame(root, text='Parameters', padx=8, pady=8)
        param_frame.pack(fill='x', padx=8, pady=6)

        self.alt_var = tk.StringVar(value='1.3')
        self.gx_var = tk.StringVar(value='0.0')
        self.gy_var = tk.StringVar(value='0.0')
        self.galt_var = tk.StringVar(value='1.5')

        self._row(param_frame, 0, 'Hover alt (m)', self.alt_var,
                  lambda: self._set_float('rlapp.targetAlt', self.alt_var))
        self._row(param_frame, 1, 'Gate origin X', self.gx_var,
                  lambda: self._set_float('rlapp.gateOriginX', self.gx_var))
        self._row(param_frame, 2, 'Gate origin Y', self.gy_var,
                  lambda: self._set_float('rlapp.gateOriginY', self.gy_var))
        self._row(param_frame, 3, 'Gate alt (m)', self.galt_var,
                  lambda: self._set_float('rlapp.gateAlt', self.galt_var))

        # Telemetry -----------------------------------------------------------
        tele_frame = tk.LabelFrame(root, text='Telemetry', padx=8, pady=8)
        tele_frame.pack(fill='both', expand=True, padx=8, pady=6)

        self._tele(tele_frame, 'Firmware state', self.live['state'])
        self._tele(tele_frame, 'Target gate',    self.live['tgtGate'])
        self._tele(tele_frame, 'Battery',        self.live['vbat'])
        self._tele(tele_frame, 'NN out',         self.live['nn'])
        self._tele(tele_frame, 'Position',       self.live['pos'])

        # Estimator reset -----------------------------------------------------
        tk.Button(root, text='Reset estimator',
                  command=lambda: Thread(target=reset_estimator, args=(cf,),
                                         daemon=True).start()).pack(pady=6)

        root.protocol('WM_DELETE_WINDOW', self.on_close)

    @staticmethod
    def _row(parent, r, label, var, cb):
        tk.Label(parent, text=label).grid(row=r, column=0, sticky='w', pady=2)
        tk.Entry(parent, textvariable=var, width=10).grid(row=r, column=1, padx=4)
        tk.Button(parent, text='Set', command=cb).grid(row=r, column=2, padx=4)

    @staticmethod
    def _tele(parent, label, var):
        row = tk.Frame(parent)
        row.pack(fill='x')
        tk.Label(row, text=label, width=16, anchor='w').pack(side='left')
        tk.Label(row, textvariable=var, anchor='w', font=('TkFixedFont',)).pack(side='left')

    def set_state(self, s):
        print(f'-> state {s} ({STATE_NAMES[s]})')
        self.cf.param.set_value('rlapp.targetState', str(s))

    def kill(self):
        # Runs each step in its own try so one failing call cannot block the rest.
        print('!!! KILL !!!')
        Thread(target=self._kill_worker, daemon=True).start()

    def _kill_worker(self):
        cf = self.cf
        for desc, fn in (
            ('targetState=IDLE', lambda: cf.param.set_value('rlapp.targetState', str(STATE_IDLE))),
            ('motorPowerSet.enable=0', lambda: cf.param.set_value('motorPowerSet.enable', '0')),
            ('stop_setpoint',   lambda: cf.commander.send_stop_setpoint()),
            ('disarm',          lambda: cf.platform.send_arming_request(False)),
        ):
            try:
                fn()
            except Exception as e:
                print(f'kill: {desc} failed: {e}')

    def _set_float(self, name, var):
        try:
            v = float(var.get())
        except ValueError:
            print(f'Invalid number for {name}')
            return
        print(f'{name} = {v}')
        self.cf.param.set_value(name, str(v))

    def update_rlapp(self, data):
        s = int(data.get('rlapp.state', 0))
        self.live['state'].set(f'{s} ({STATE_NAMES.get(s, "?")})')
        self.live['tgtGate'].set(str(int(data.get('rlapp.tgtGate', 0))))
        self.live['nn'].set('{:+.2f} {:+.2f} {:+.2f} {:+.2f}'.format(
            data.get('rlapp.nnOut0', 0.0), data.get('rlapp.nnOut1', 0.0),
            data.get('rlapp.nnOut2', 0.0), data.get('rlapp.nnOut3', 0.0)))

    def update_state(self, data):
        self.live['pos'].set('x={:+.2f} y={:+.2f} z={:+.2f}'.format(
            data.get('stateEstimate.x', 0.0),
            data.get('stateEstimate.y', 0.0),
            data.get('stateEstimate.z', 0.0)))
        self.live['vbat'].set(f"{data.get('pm.vbat', 0.0):.2f} V")

    def on_close(self):
        try:
            self.cf.param.set_value('rlapp.targetState', str(STATE_IDLE))
        except Exception:
            pass
        self.root.destroy()


# ---- Logging hooks ---------------------------------------------------------
def attach_logs(cf, gui):
    rl_log = LogConfig(name='rlapp', period_in_ms=100)
    for v in ('rlapp.state', 'rlapp.tgtGate'):
        rl_log.add_variable(v, 'uint8_t')
    for v in ('rlapp.nnOut0', 'rlapp.nnOut1', 'rlapp.nnOut2', 'rlapp.nnOut3'):
        rl_log.add_variable(v, 'float')
    rl_log.data_received_cb.add_callback(lambda ts, d, lc: gui.update_rlapp(d))
    cf.log.add_config(rl_log)
    rl_log.start()

    state_log = LogConfig(name='state', period_in_ms=200)
    for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z'):
        state_log.add_variable(v, 'float')
    state_log.add_variable('pm.vbat', 'float')
    state_log.data_received_cb.add_callback(lambda ts, d, lc: gui.update_state(d))
    cf.log.add_config(state_log)
    state_log.start()
    return rl_log, state_log


def connection_failed(link_uri, msg):
    print(f'Connection to {link_uri} failed: {msg}')


# ---- Main ------------------------------------------------------------------
def main():
    print('initializing drivers')
    cflib.crtp.init_drivers()

    mocap_wrapper = MocapWrapper(rigid_body_name)

    print(f'Connecting to {uri}')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.connection_lost.add_callback(connection_failed)

        mocap_wrapper.on_pose = lambda p: send_extpose_quat(cf, p[0], p[1], p[2], p[3])

        print('Activating Kalman estimator')
        activate_kalman_estimator(cf)
        reset_estimator(cf)

        # Force IDLE on connect — never inherit a stale state from a prior run.
        cf.param.set_value('rlapp.targetState', str(STATE_IDLE))
        cf.platform.send_arming_request(True)

        root = tk.Tk()
        gui = RLControlGui(root, cf)
        rl_log, state_log = attach_logs(cf, gui)

        try:
            root.mainloop()
        finally:
            try:
                rl_log.stop()
                state_log.stop()
            except Exception:
                pass

    mocap_wrapper.close()


if __name__ == '__main__':
    main()
