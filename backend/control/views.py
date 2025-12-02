from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
from django.shortcuts import get_object_or_404
from .models import Robot
from .serializers import RobotSerializer
from .services.ros import ROSClient

# ===== Robots list =====
class RobotListView(APIView):
    def get(self, request):
        data = RobotSerializer(Robot.objects.all(), many=True).data
        return Response(data)

# ===== Connect =====
class ConnectView(APIView):
    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        addr = request.data.get("addr", "")
        client = ROSClient(robot_id)
        result = client.connect(addr)
        robot.addr = addr
        robot.save(update_fields=["addr"])
        return Response({"ok": True, **result}, status=200)

class RobotStatusView(APIView):
    def get(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        client = ROSClient(robot_id)

        # Lấy JSON từ Flask /status qua ROSClient
        try:
            s = client.get_status() or {}
        except Exception as e:
            print("[RobotStatusView] get_status error:", e)
            s = {}

        changed_fields = []

        # Cập nhật battery / fps vào DB nếu có
        battery = s.get("battery")
        if battery is not None and hasattr(robot, "battery"):
            robot.battery = battery
            changed_fields.append("battery")

        fps = s.get("fps")
        if fps is not None and hasattr(robot, "fps"):
            robot.fps = fps
            changed_fields.append("fps")

        robot_connected = s.get("robot_connected")
        if robot_connected is not None and hasattr(robot, "status_text"):
            robot.status_text = "online" if robot_connected else "offline"
            changed_fields.append("status_text")

        if changed_fields:
            robot.save(update_fields=changed_fields)

        # dữ liệu cơ bản của Robot (id, name, battery, fps, ...)
        data = RobotSerializer(robot).data

        # Gắn thêm block telemetry cho frontend
        data["telemetry"] = {
            "robot_connected": s.get("robot_connected", False),
            "turn_speed_range": s.get("turn_speed_range"),
            "step_default": s.get("step_default"),
            "z_range": s.get("z_range"),
            "z_current": s.get("z_current"),
            "pitch_range": s.get("pitch_range"),
            "pitch_current": s.get("pitch_current"),
            "battery": s.get("battery"),
            "fw": s.get("fw"),
            "fps": s.get("fps"),
            "system": s.get("system"),  # <-- QUAN TRỌNG
        }

        return Response(data, status=200)

# ===== FPV =====
class FPVView(APIView):
    def get(self, request, robot_id):
        client = ROSClient(robot_id)
        return Response({"stream_url": client.get_fpv_url()})

# ===== Commands =====
class SpeedModeView(APIView):
    def post(self, request, robot_id):
        mode = request.data.get("mode")  # "slow"|"normal"|"high"
        ROSClient(robot_id).set_speed_mode(mode)
        return Response({"ok": True})

class MoveCommandView(APIView):
    def post(self, request, robot_id):
        """
        Body JSON:
        {
          "vx": 0.1, "vy": 0.0, "vz": 0.0,
          "rx": 0.0, "ry": 0.0, "rz": 0.3
        }
        """
        ROSClient(robot_id).move(request.data)
        return Response({"ok": True})

class PostureView(APIView):
    def post(self, request, robot_id):
        # name: "Lie_Down" | "Stand_Up" | "Sit_Down" | "Squat" | "Crawl"
        ROSClient(robot_id).posture(request.data.get("name"))
        return Response({"ok": True})

class BehaviorView(APIView):
    def post(self, request, robot_id):
        # name: "Wave_Hand" | "Handshake" | ...
        ROSClient(robot_id).behavior(request.data.get("name"))
        return Response({"ok": True})

class LidarView(APIView):
    def post(self, request, robot_id):
        # action: "start" | "stop"
        ROSClient(robot_id).lidar(request.data.get("action"))
        return Response({"ok": True})

class BodyAdjustView(APIView):
    def post(self, request, robot_id):
        """
        Body JSON:
        { "tx": 0, "ty": 0, "tz": 0, "rx": 0, "ry": 0, "rz": 0 }
        """
        ROSClient(robot_id).body_adjust(request.data)
        return Response({"ok": True})
    
class StabilizingModeView(APIView):
    def post(self, request, robot_id):
        action = request.data.get("action")  
        ROSClient(robot_id).stabilizing_mode(action)
        return Response({"ok": True})
