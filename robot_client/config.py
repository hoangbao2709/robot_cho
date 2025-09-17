# -*- coding: utf-8 -*-

import os

import os
from pymongo import MongoClient

MONGODB_URI = os.environ.get("MONGODB_URI", "mongodb+srv://hoangbao27092004_db_user:baozzz123@cluster0.q7fvyyw.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0")

client = MongoClient(MONGODB_URI, tlsAllowInvalidCertificates=True)
col = client["robot"]["tunnels"]

doc = col.find_one({"device_id": "rpi5-dogzilla"}, sort=[("updated_at", -1)])
if not doc or "url" not in doc:
    raise RuntimeError("Không tìm thấy URL trong MongoDB")

BASE_URL = doc["url"].rstrip("/")
print("BASE_URL =", BASE_URL)


API_KEY = os.environ.get("DOGZILLA_API_KEY", None)

REQUEST_HEADERS = {
    "Content-Type": "application/json",
    **({"X-API-Key": API_KEY} if API_KEY else {}),
}

VERIFY_SSL = os.environ.get("DOGZILLA_VERIFY_SSL", "true").lower() == "true"

REQUEST_TIMEOUT = int(os.environ.get("DOGZILLA_TIMEOUT", "5")) 

CONTROL_URL = f"{BASE_URL}/control"
CAMERA_URL  = f"{BASE_URL}/camera"
STATUS_URL  = f"{BASE_URL}/status"
HEALTH_URL  = f"{BASE_URL}/health" 

# ========= UI / điều khiển =========
UI_FPS_MS = 33          

TURN_SPEED = 40
DEADZONE   = 5

TURN_SPEED_MIN = 15
TURN_SPEED_MAX = 70
SCALE_PX       = 60
SPEED_GAMMA    = 0.8
SPEED_SMOOTH_ALPHA = 0.5
SPEED_UPDATE_DELTA = 2

HOLD_MS = 120
MOUSELOOK_HZ = 60
REPEATER_HZ  = 12

# ========= Z / posture =========
Z_MIN      = 75
Z_MAX      = 110
Z_INITIAL  = 105
SCROLL_Z_STEP = 1

# ========= Attitude =========
ATTITUDE_CMD       = "attitude"
ATTITUDE_AXIS_KEY  = "axis"
ATTITUDE_VALUE_KEY = "value"
AXIS_ROLL  = "r"
AXIS_PITCH = "p"
AXIS_YAW   = "y"

PITCH_MIN = -15
PITCH_MAX =  15
PITCH_INITIAL = 0
PITCH_DEADZONE_PIX   = 3
PITCH_SCALE_PY       = 60
PITCH_GAMMA          = 1.0
PITCH_MAX_STEP_DEG   = 1.2
PITCH_SMOOTH_ALPHA   = 0.5
PITCH_UPDATE_DELTA   = 1

ROLL_MIN = -20
ROLL_MAX =  20
YAW_MIN  = -11
YAW_MAX  =  11

ATT_HOLD_HZ = 60
ROLL_YAW_RATE_DPS = 40.0
ATT_UPDATE_DELTA  = 1.0
