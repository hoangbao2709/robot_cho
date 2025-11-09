# Đây là lớp “adapter” để tách Django khỏi ROS2.
# Bạn thay nội dung TODO để gọi ROS 2 (rclpy, rosbridge, custom service, …)

from typing import Dict, Any

class ROSClient:
    def __init__(self, robot_id: str):
        self.robot_id = robot_id

    # ===== Telemetry =====
    def get_status(self) -> Dict[str, Any]:
        # TODO: đọc từ ROS topics/service
        return {
            "location": {"lat": 25.23234, "lon": 19.76543},
            "cleaning_progress": 80,
            "floor": "1st",
            "status": "Resting",
            "water_level": 50,
            "battery": 85,
            "fps": 30,
        }

    # ===== Commands =====
    def connect(self, addr: str) -> Dict[str, Any]:
        # TODO: init node/bridge đến robot
        return {"connected": True, "addr": addr}

    def get_fpv_url(self) -> str:
        # TODO: trả URL stream (WebRTC/MJPEG/RTSP gateway)
        return f"http://127.0.0.1:8080/stream/{self.robot_id}"

    def set_speed_mode(self, mode: str) -> None:
        assert mode in ("slow", "normal", "high")

    def move(self, payload: Dict[str, Any]) -> None:
        # payload: vx, vy, vz, rx, ry, rz  (m/s, rad/s)
        pass

    def posture(self, name: str) -> None:
        # e.g. Stand_Up, Lie_Down, ...
        pass

    def behavior(self, name: str) -> None:
        # e.g. Wave_Hand, Handshake, ...
        pass

    def lidar(self, action: str) -> None:
        assert action in ("start", "stop")

    def body_adjust(self, sliders: Dict[str, float]) -> None:
        # sliders: {"tx":..., "ty":..., "tz":..., "rx":..., "ry":..., "rz":...}
        pass
