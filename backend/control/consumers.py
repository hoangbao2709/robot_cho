# control/consumers.py
# -*- coding: utf-8 -*-
import json
import time
import math

from channels.generic.websocket import WebsocketConsumer
from django.apps import apps

# Tham số giống MouseLook.py
DEADZONE = 6                   # px
SCALE_PX = 120                 # px để đạt max speed
SPEED_GAMMA = 1.6              # cong đáp ứng
SPEED_SMOOTH_ALPHA = 0.35      # 0..1
TURN_SPEED_MIN = 15
TURN_SPEED_MAX = 70
SPEED_UPDATE_DELTA = 3
PITCH_DEADZONE_PIX = 4
PITCH_SCALE_PY = 160
PITCH_GAMMA = 1.4
PITCH_MAX_STEP_DEG = 2.5
PITCH_MIN, PITCH_MAX = -25, 25
PITCH_UPDATE_DELTA = 1.0
HOLD_MS = 180

# cache client theo robot
_clients = {}

DRY_RUN = True

def get_robot_client(robot):
    """
    Trả về RobotClient đã bọc Dogzilla client.
    """
    from .services.robot_client_adapter import RobotClient
    key = robot.id
    cli = _clients.get(key)
    if cli is None:
        cli = RobotClient(robot)
        _clients[key] = cli
    return cli


class ControlInputConsumer(WebsocketConsumer):
    """
    FE gửi qua WS (ws://.../ws/robots/<robot_id>/control/):

      { "type": "ml_move",   "dx": number, "dy": number }
      { "type": "ml_enable", "enable": true|false }
      { "type": "stop" }

    Backend sẽ:
      - dx -> quay trái/phải (start_motion("turnleft"/"turnright"))
      - dy -> chỉnh pitch (set_pitch)
    """

    def connect(self):
        # Lấy robot_id từ URL
        self.robot_id = self.scope["url_route"]["kwargs"]["robot_id"]

        Robot = apps.get_model("control", "Robot")
        self.robot, _ = Robot.objects.get_or_create(
            id=self.robot_id,
            defaults={"name": self.robot_id},
        )

        # RobotClient bọc Dogzilla client thật
        self.client = get_robot_client(self.robot)

        # State cho mouselook
        self.enabled = True
        self.last_cmd = None
        self.last_speed = 0
        self.last_move_ts = 0
        self.pitch_value = 0.0
        self.last_sent_pitch = float("nan")

        self.accept()
        self.send_json({"type": "hello", "robot": self.robot_id})

    def disconnect(self, close_code):
        try:
            self.client.stop()
        except Exception:
            pass

    def receive(self, text_data=None, bytes_data=None):
        # Nhận JSON từ frontend
        try:
            msg = json.loads(text_data or "{}")
        except Exception:
            return

        t = msg.get("type")
        print("[WS] receive:", msg)  
        if t == "ml_enable":
            self.enabled = bool(msg.get("enable", True))
            if not self.enabled:
                self._send_stop()
            self.send_json({"type": "ml_enabled", "enabled": self.enabled})
            return

        if t == "stop":
            self._send_stop()
            return

        if t == "ml_move" and self.enabled:
            dx = float(msg.get("dx", 0.0))
            dy = float(msg.get("dy", 0.0))
            print(f"[WS] ml_move dx={dx} dy={dy}")   # <-- LOG 3: dx,dy
            self._apply_dx_turn(dx)
            self._apply_dy_pitch(dy)

    # ----------------- helpers -----------------

    def _send_stop(self):
        if self.last_cmd != "stop":
            try:
                self.client.stop()
            except Exception:
                pass
            self.last_cmd = "stop"
            self.last_speed = 0

    def _apply_dx_turn(self, dx: float):
        now_ms = int(time.time() * 1000)
        absdx = abs(dx)

        # deadzone -> có HOLD_MS để tránh dừng quá gấp
        if absdx < DEADZONE:
            if (
                self.last_cmd in ("turnleft", "turnright")
                and (now_ms - self.last_move_ts) < HOLD_MS
            ):
                return
            self._send_stop()
            return

        self.last_move_ts = now_ms

        # chuẩn hoá 0..1 và bẻ cong đáp ứng
        norm = min(1.0, absdx / float(SCALE_PX))
        curved = norm ** SPEED_GAMMA
        target = TURN_SPEED_MIN + (TURN_SPEED_MAX - TURN_SPEED_MIN) * curved

        # smoothing
        if self.last_speed == 0:
            smoothed = target
        else:
            a = SPEED_SMOOTH_ALPHA
            smoothed = a * target + (1 - a) * self.last_speed

        speed_int = int(
            round(
                min(TURN_SPEED_MAX, max(TURN_SPEED_MIN, smoothed))
            )
        )
        direction = "turnleft" if dx < 0 else "turnright"

        # tránh spam lệnh nếu không thay đổi đáng kể
        if (
            direction == self.last_cmd
            and abs(speed_int - self.last_speed) < SPEED_UPDATE_DELTA
        ):
            return

        # gửi lệnh quay qua RobotClient
        try:
            # RobotClient bên bạn bọc Dogzilla client trong .cli
            self.client.cli.start_motion(direction, speed=speed_int)
        except Exception as e:
            print("start_motion error:", e)

        self.last_cmd = direction
        self.last_speed = speed_int

    def _apply_dy_pitch(self, dy: float):
        absdy = abs(dy)
        if absdy < PITCH_DEADZONE_PIX:
            return

        norm = min(1.0, absdy / float(PITCH_SCALE_PY))
        curved = norm ** PITCH_GAMMA
        step = curved * PITCH_MAX_STEP_DEG
        delta = (-step) if dy < 0 else (+step)  # kéo chuột lên = ngẩng

        new_pitch = self.pitch_value + delta
        a = 0.35
        smoothed = a * new_pitch + (1 - a) * self.pitch_value
        smoothed = max(PITCH_MIN, min(PITCH_MAX, smoothed))

        # chỉ gửi nếu thay đổi đủ lớn
        if (
            self.last_sent_pitch != self.last_sent_pitch  # NaN check
            or abs(smoothed - self.last_sent_pitch) >= PITCH_UPDATE_DELTA
        ):
            try:
                self.client.cli.set_pitch(int(round(smoothed)))
            except Exception as e:
                print("set_pitch error:", e)
            self.last_sent_pitch = smoothed

        self.pitch_value = smoothed

    # helper gửi json
    def send_json(self, data):
        self.send(text_data=json.dumps(data))
