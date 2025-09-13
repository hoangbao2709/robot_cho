# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import messagebox
import requests
import threading
import cv2
from PIL import Image, ImageTk
import time

# --- CONFIG ---
CONTROL_URL = "http://192.168.137.195:9000/control"
CAMERA_URL  = "http://192.168.137.195:9000/camera"
REQUEST_TIMEOUT = 3
UI_FPS_MS = 33      # ~30 FPS
TURN_SPEED = 40     # [-70,70] tùy FW
DEADZONE   = 5      # pixel

# --- HTTP helpers ---
def _post_payload(payload: dict):
    try:
        resp = requests.post(CONTROL_URL, json=payload, timeout=REQUEST_TIMEOUT)
        print(payload, "->", resp.text)
    except requests.exceptions.RequestException as e:
        print("Connection error:", e)
        try:
            messagebox.showerror("Error", f"Failed to send {payload}: {e}")
        except Exception:
            pass

# Gửi lặp đều lệnh hiện hành (forward/turnleft...), ~12Hz
class CommandRepeater:
    def __init__(self, hz=12):
        self.hz = hz
        self._target = None          # tuple(command, kwargs)
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
                if self._target is not None:
                    target = self._target
            if target is not None:
                cmd, kw = target
                payload = {"command": cmd}
                payload.update(kw)
                _post_payload(payload)
            time.sleep(1.0 / self.hz)

repeater = CommandRepeater(hz=12)

def start_motion(command: str, **kwargs):
    """Bắt đầu gửi lặp đều lệnh (tới khi stop)."""
    repeater.set(command, **kwargs)

def stop_command():
    """Dừng chuyển động và dừng lặp."""
    repeater.clear()
    _post_payload({"command": "stop"})

# --- CAMERA READER (thread) + UI updater (main thread) ---
running = True
_latest_bgr = None
_frame_lock = threading.Lock()

def camera_reader():
    global running, _latest_bgr
    while running:
        cap = cv2.VideoCapture(CAMERA_URL)
        if not cap.isOpened():
            print("Cannot open camera stream, retry in 1s...")
            time.sleep(1.0)
            continue
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        while running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.02)
                continue
            with _frame_lock:
                _latest_bgr = frame
        cap.release()

def ui_update():
    global _latest_bgr
    if not running:
        return
    frame = None
    with _frame_lock:
        if _latest_bgr is not None:
            frame = _latest_bgr
    if frame is not None:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        imgtk = ImageTk.PhotoImage(image=img)
        lbl_img.imgtk = imgtk
        lbl_img.configure(image=imgtk)
    root.after(UI_FPS_MS, ui_update)

# --- WINDOW EVENTS ---
def on_closing():
    global running
    running = False
    try:
        stop_command()
    except Exception:
        pass
    root.destroy()

# --- UI SETUP ---
root = tk.Tk()
root.title("Dogzilla Control Panel (PUBG-style)")
root.geometry("900x720")

lbl_img = tk.Label(root)
lbl_img.pack(pady=10)

pad = 10
panel = tk.Frame(root)
panel.pack(pady=10)

# UI ready flag
ui_ready = False

def enable_ui():
    global ui_ready
    ui_ready = True
    print("UI is now ready (hover enabled).")

# --- Hover buttons (WASD-like) ---
def make_hover_button(parent, text, command_name, width=14, with_speed=False):
    btn = tk.Button(parent, text=text, width=width)
    def go(_=None):
        if ui_ready:
            if with_speed:
                start_motion(command_name, speed=TURN_SPEED)
            else:
                start_motion(command_name)
    btn.bind("<Enter>",           go)
    btn.bind("<Leave>",           lambda e: stop_command())
    btn.bind("<ButtonPress-1>",   go)
    btn.bind("<ButtonRelease-1>", lambda e: stop_command())
    return btn

# --- Keyboard WASD + CTRL toggle mouse look ---
pressed_keys = set()

# Toggle state control for CTRL (avoid auto-repeat)
ctrl_down = False

# --- PUBG-style mouse look (polling + warp-to-center, FPS-style) ---
mouse_look_enabled = True
center_x = None
center_y = None
last_mouse_cmd = None  # None | "turnleft" | "turnright" | "stop"
HOLD_MS = 120
_last_move_ts = 0


def _recalc_center():
    root.update_idletasks()
    cx = root.winfo_rootx() + root.winfo_width() // 2
    cy = root.winfo_rooty() + root.winfo_height() // 2
    return cx, cy


def _warp_pointer_to_center():
    """Đưa con trỏ về đúng tâm cửa sổ (đa nền tảng nếu có thể)."""
    global center_x, center_y
    # Thử Tk warp (tọa độ widget) – chạy tốt trên nhiều bản Tk
    try:
        root.event_generate("<Motion>", warp=True,
                            x=root.winfo_width()//2,
                            y=root.winfo_height()//2)
        return True
    except Exception:
        pass

    # Fallback OS-level (best effort)
    try:
        import platform
        sysname = platform.system()
        if sysname == "Windows":
            import ctypes
            ctypes.windll.user32.SetCursorPos(center_x, center_y)
            return True
        elif sysname == "Darwin":
            try:
                import Quartz  # type: ignore
                Quartz.CGWarpMouseCursorPosition((center_x, center_y))
                Quartz.CGAssociateMouseAndMouseCursorPosition(True)
                return True
            except Exception:
                pass
        # Linux/X11 thường đã OK với Tk warp ở trên
    except Exception:
        pass
    return False


def enable_mouse_look():
    global mouse_look_enabled, center_x, center_y
    mouse_look_enabled = True
    root.config(cursor="none")
    center_x, center_y = _recalc_center()
    _warp_pointer_to_center()
    print("Mouse look: ENABLED (center-locked)")


def disable_mouse_look():
    global mouse_look_enabled
    mouse_look_enabled = False
    root.config(cursor="")
    print("Mouse look: DISABLED")


def _apply_mouse_dx(dx: int):
    global last_mouse_cmd, _last_move_ts
    if not ui_ready:
        return
    now = int(time.time() * 1000)

    if abs(dx) < DEADZONE:
        # Giữ lệnh cũ một nhịp nhỏ để đỡ "giật"
        if last_mouse_cmd in ("turnleft", "turnright") and (now - _last_move_ts) < HOLD_MS:
            return
        if last_mouse_cmd != "stop":
            stop_command()
            last_mouse_cmd = "stop"
        return

    _last_move_ts = now

    if dx < 0:
        if last_mouse_cmd != "turnleft":
            start_motion("turnleft", speed=TURN_SPEED)
            last_mouse_cmd = "turnleft"
    else:
        if last_mouse_cmd != "turnright":
            start_motion("turnright", speed=TURN_SPEED)
            last_mouse_cmd = "turnright"


def mouse_look_tick():
    """Đọc delta so với TÂM cửa sổ và warp lại về TÂM mỗi frame (~60Hz)."""
    global center_x, center_y
    if mouse_look_enabled and ui_ready and root.winfo_exists():
        if center_x is None:
            center_x, center_y = _recalc_center()
        try:
            px = root.winfo_pointerx()
        except Exception:
            px = None
        if px is not None:
            dx = px - center_x
            _apply_mouse_dx(dx)
        _warp_pointer_to_center()
    root.after(16, mouse_look_tick)


def on_configure(_event):
    # Khi đổi kích thước/vị trí: tính lại tâm để warp đúng
    global center_x, center_y
    center_x, center_y = _recalc_center()


# --- Layout ---
grid = tk.Frame(panel)
grid.pack()

btn_w = make_hover_button(grid, "W (Forward)", "forward")
btn_a = make_hover_button(grid, "A (Left)",    "left")
btn_s = make_hover_button(grid, "S (Back)",    "back")
btn_d = make_hover_button(grid, "D (Right)",   "right")

tk.Label(grid, text="").grid(row=0, column=0, padx=pad, pady=pad)
btn_w.grid(row=0, column=1, padx=pad, pady=pad)
tk.Label(grid, text="").grid(row=0, column=2, padx=pad, pady=pad)
btn_a.grid(row=1, column=0, padx=pad, pady=pad)
tk.Label(grid, text="").grid(row=1, column=1, padx=pad, pady=pad)
btn_d.grid(row=1, column=2, padx=pad, pady=pad)
tk.Label(grid, text="").grid(row=2, column=0, padx=pad, pady=pad)
btn_s.grid(row=2, column=1, padx=pad, pady=pad)
tk.Label(grid, text="").grid(row=2, column=2, padx=pad, pady=pad)

# Thêm nút quay (giữ chuột/hover để quay liên tục)
btn_turn_l = make_hover_button(panel, "Turn Left",  "turnleft",  with_speed=True)
btn_turn_r = make_hover_button(panel, "Turn Right", "turnright", with_speed=True)
btn_turn_l.pack(side="left", padx=pad)
btn_turn_r.pack(side="left", padx=pad)

btn_stop = tk.Button(panel, text="STOP", width=14, command=stop_command)
btn_stop.pack(pady=(pad, 0))

def on_key_press(event):
    global ctrl_down
    key = event.keysym

    # CTRL: TOGGLE mouse look on PRESS (không giữ), bỏ qua auto-repeat
    if key in ("Control_L", "Control_R"):
        if not ctrl_down:
            ctrl_down = True
            if mouse_look_enabled:
                disable_mouse_look()
                stop_command()
            else:
                enable_mouse_look()
        return

    if not ui_ready:
        return

    k = key.lower()
    if k not in pressed_keys:
        pressed_keys.add(k)
        if k == "w":
            start_motion("forward")
        elif k == "s":
            start_motion("back")
        elif k == "a":
            start_motion("left")
        elif k == "d":
            start_motion("right")

def on_key_release(event):
    global ctrl_down
    key = event.keysym

    # nhả CTRL: chỉ hạ cờ, không toggle
    if key in ("Control_L", "Control_R"):
        ctrl_down = False
        return

    k = key.lower()
    if k in pressed_keys:
        pressed_keys.remove(k)
        stop_command()


# --- Bindings ---
root.bind("<KeyPress>",   lambda e: on_key_press(e))
root.bind("<KeyRelease>", lambda e: on_key_release(e))

# Không cần <Motion> nữa, nhưng vẫn có thể để dự phòng nếu bạn muốn:
# root.bind_all("<Motion>", lambda e: None)
root.bind("<Configure>",  on_configure)
root.bind_all("<ButtonRelease-1>", lambda e: stop_command())

# Gửi stop ngay khi mở
try:
    stop_command()
except Exception:
    pass

# Camera & UI
threading.Thread(target=camera_reader, daemon=True).start()
root.after(100, ui_update)

# Kích hoạt hover sau 700ms; mặc định bật mouse look (có thể tắt bằng CTRL)
root.after(700,  enable_ui)
root.after(1000, enable_mouse_look)
# Polling mouse look (60Hz)
root.after(1200, mouse_look_tick)

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
