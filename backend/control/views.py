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

# ===== Status =====
class RobotStatusView(APIView):
    def get(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        client = ROSClient(robot_id)
        s = client.get_status()

        robot.location_lat = s["location"]["lat"]
        robot.location_lon = s["location"]["lon"]
        robot.cleaning_progress = s["cleaning_progress"]
        robot.floor = s["floor"]
        robot.status_text = s["status"]
        robot.water_level = s["water_level"]
        robot.battery = s["battery"]
        robot.fps = s["fps"]
        robot.save()

        return Response(RobotSerializer(robot).data, status=200)

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
