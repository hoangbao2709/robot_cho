# control/services/robot_client_adapter.py

from .ros import ROSClient


class _DummyCLI:
    """
    Lớp giả lập CLI robot để dùng chung API với code Tkinter cũ:
      - start_motion(direction, speed)
      - set_pitch(pitch_deg)
      - stop()
    Ở đây mình chỉ log + có thể gửi lệnh move() 0 vận tốc khi stop.
    """

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.ros = ROSClient(robot_id)

    def start_motion(self, direction: str, speed: int):
        # TODO: map sang move() thực tế nếu muốn
        print(f"[DummyCLI] start_motion dir={direction} speed={speed}")

        # ví dụ: quay trái/phải bằng yaw (rz)
        if direction == "turnleft":
            rz = +0.8
        elif direction == "turnright":
            rz = -0.8
        else:
            rz = 0.0

        try:
            self.ros.move({"vx": 0.0, "vy": 0.0, "vz": 0.0,
                           "rx": 0.0, "ry": 0.0, "rz": rz})
        except Exception as e:
            print("[DummyCLI] move error:", e)

    def set_pitch(self, pitch_deg: int):
        # chỗ này chỉ log để test; sau này map sang API camera/servo
        print(f"[DummyCLI] set_pitch {pitch_deg}°")

    def stop(self):
        print("[DummyCLI] stop()")
        try:
            self.ros.move({"vx": 0.0, "vy": 0.0, "vz": 0.0,
                           "rx": 0.0, "ry": 0.0, "rz": 0.0})
        except Exception as e:
            print("[DummyCLI] stop move error:", e)


class RobotClient:
    """
    Wrapper giống như bên app Tkinter:
      - có thuộc tính .cli
      - có method stop()
    """

    def __init__(self, robot):
        # robot là instance model Robot
        self.robot = robot
        self.cli = _DummyCLI(robot.id)

    def stop(self):
        self.cli.stop()
