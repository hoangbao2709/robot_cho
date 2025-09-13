# -*- coding: utf-8 -*-
import threading
import time
import requests
from . import config

class CommandRepeater:
    def __init__(self, hz=config.REPEATER_HZ):
        self.hz = hz
        self._target = None      
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def set(self, command: str, **kwargs):
        with self._lock:
            self._target = (command, kwargs)

    def clear(self):
        with self._lock:
            self._target = None

    def _loop(self):
        while self._running:
            target = None
            with self._lock:
                target = self._target
            if target is not None:
                cmd, kw = target
                payload = {"command": cmd}
                payload.update(kw)
                self._post_payload(payload)
            time.sleep(1.0 / self.hz)

    @staticmethod
    def _post_payload(payload: dict):
        try:
            resp = requests.post(config.CONTROL_URL, json=payload, timeout=config.REQUEST_TIMEOUT)
            print(payload, "->", resp.text)
            return resp.status_code, resp.text
        except requests.exceptions.RequestException as e:
            print("Connection error:", e)
            return None, str(e)

class Control:
    def __init__(self):
        self.repeater = CommandRepeater()

    def start_motion(self, command: str, **kwargs):
        self.repeater.set(command, **kwargs)

    def stop(self):
        self.repeater.clear()
        self.repeater._post_payload({"command": "stop"})

    def set_z(self, z: int):
        payload = {"command": "setz", "value": int(z)}
        self.repeater._post_payload(payload)

    def set_attitude(self, axis: str, value: float):
        payload = {
            "command": config.ATTITUDE_CMD,
            config.ATTITUDE_AXIS_KEY: axis,
            config.ATTITUDE_VALUE_KEY: float(value),
        }
        code, _ = self.repeater._post_payload(payload)
        if code is None or code >= 400:
            if axis == config.AXIS_ROLL:
                self.repeater._post_payload({"command": "setroll", "value": float(value)})
            elif axis == config.AXIS_PITCH:
                self.repeater._post_payload({"command": "setpitch", "value": float(value)})
            elif axis == config.AXIS_YAW:
                self.repeater._post_payload({"command": "setyaw", "value": float(value)})

    def set_pitch(self, pitch_deg: float):
        self.set_attitude(config.AXIS_PITCH, pitch_deg)

    def set_roll(self, roll_deg: float):
        self.set_attitude(config.AXIS_ROLL, roll_deg)

    def set_yaw(self, yaw_deg: float):
        self.set_attitude(config.AXIS_YAW, yaw_deg)

    def set_roll_yaw(self, roll_deg: float, yaw_deg: float):
        self.set_roll(roll_deg)
        self.set_yaw(yaw_deg)
