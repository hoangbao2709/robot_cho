# -*- coding: utf-8 -*-
from flask import Blueprint, request
from ..robot import robot

bp = Blueprint("control", __name__)

@bp.route("/control", methods=["POST"])
def control():
    data = request.get_json(silent=True) or {}
    cmd = str(data.get("command", "")).lower().strip()

    raw_step  = data.get("step")
    raw_speed = data.get("speed")
    raw_value = data.get("value")  # setz / setroll / setyaw / setpitch
    raw_delta = data.get("delta")  # adjustz (nếu dùng)

    # parse ints for step/speed
    step = None
    if raw_step is not None:
        try: step = int(raw_step)
        except Exception:
            return ("invalid step", 400, {"Content-Type": "text/plain; charset=utf-8"})

    speed = None
    if raw_speed is not None:
        try: speed = int(raw_speed)
        except Exception:
            return ("invalid speed", 400, {"Content-Type": "text/plain; charset=utf-8"})

    # ----- dispatch -----
    if cmd in ("forward", "back", "left", "right", "turnleft", "turnright", "stop"):
        res = robot.do_motion(cmd, step=step, speed=speed)

    # Z
    elif cmd == "setz":
        if raw_value is None:
            return ("missing value", 400, {"Content-Type": "text/plain; charset=utf-8"})
        res = robot.setz(raw_value)
    elif cmd == "adjustz":
        if raw_delta is None:
            return ("missing delta", 400, {"Content-Type": "text/plain; charset=utf-8"})
        res = robot.adjustz(raw_delta)

    # --- Attitude (generic) ---
    elif cmd == "attitude":
        axis  = data.get("axis")
        value = data.get("value")
        if axis is None or value is None:
            return ("missing axis/value", 400, {"Content-Type": "text/plain; charset=utf-8"})
        res = robot.set_attitude(axis, value)

    # --- Per-axis helpers (client fallback) ---
    elif cmd == "setroll":
        if raw_value is None:
            return ("missing value", 400, {"Content-Type": "text/plain; charset=utf-8"})
        res = robot.set_roll(float(raw_value))
    elif cmd == "setyaw":
        if raw_value is None:
            return ("missing value", 400, {"Content-Type": "text/plain; charset=utf-8"})
        res = robot.set_yaw(float(raw_value))
    elif cmd == "setpitch":
        if raw_value is None:
            return ("missing value", 400, {"Content-Type": "text/plain; charset=utf-8"})
        res = robot.set_pitch(float(raw_value))

    # status passthrough
    elif cmd == "status":
        from .status import status as status_route
        return status_route()

    else:
        return (f"unknown command: {cmd}", 400, {"Content-Type": "text/plain; charset=utf-8"})

    if isinstance(res, str) and res.startswith("ok"):
        return (res, 200, {"Content-Type": "text/plain; charset=utf-8"})
    else:
        return (res, 400, {"Content-Type": "text/plain; charset=utf-8"})
