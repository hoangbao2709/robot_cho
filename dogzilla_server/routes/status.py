# -*- coding: utf-8 -*-
from flask import Blueprint, jsonify
from .. import config
from ..robot import robot

bp = Blueprint("status", __name__)

@bp.route("/status", methods=["GET", "POST"])
def status():
    s = {
        "robot_connected": robot.dog is not None,
        "turn_speed_range": [config.TURN_MIN, config.TURN_MAX],
        "step_default": config.STEP_DEFAULT,
        "z_range": [config.Z_MIN, config.Z_MAX],
        "z_current": robot.z_current(),
        # attitude info
        "roll_range":  [config.ROLL_MIN,  config.ROLL_MAX],
        "yaw_range":   [config.YAW_MIN,   config.YAW_MAX],
        "pitch_range": [config.PITCH_MIN, config.PITCH_MAX],
        "roll_current":  robot.roll_current(),
        "yaw_current":   robot.yaw_current(),
        "pitch_current": robot.pitch_current(),
    }
    if robot.dog is not None:
        try: s["battery"] = robot.dog.read_battery()
        except Exception: s["battery"] = None
        try: s["fw"] = robot.dog.read_version()
        except Exception: s["fw"] = None
    return jsonify(s)
