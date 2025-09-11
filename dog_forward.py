import tkinter as tk
from tkinter import messagebox
import requests
import threading
import cv2
from PIL import Image, ImageTk
import numpy as np

# --- CONFIG ---
CONTROL_URL = "http://192.168.5.2:9000/control"  # Flask /control endpoint on Pi
CAMERA_URL = "http://192.168.5.2:9000/camera"    # Flask /camera endpoint (MJPEG stream)

# --- SEND COMMAND TO ROBOT ---
def send_command(command, step=20):
    try:
        resp = requests.post(CONTROL_URL, json={"command": command, "step": step}, timeout=3)
        print(f"{command} ->", resp.text)
    except requests.exceptions.RequestException as e:
        print("Connection error:", e)
        messagebox.showerror("Error", f"Failed to send {command}: {e}")

# --- CAMERA STREAM HANDLER ---
def update_frame():
    global running
    cap = cv2.VideoCapture(CAMERA_URL)
    if not cap.isOpened():
        print("Cannot open camera stream")
        return
    while running:
        ret, frame = cap.read()
        if not ret:
            continue
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image=img)
        lbl_img.imgtk = imgtk
        lbl_img.configure(image=imgtk)
    cap.release()

def on_closing():
    global running
    running = False
    root.destroy()

# --- UI SETUP ---
root = tk.Tk()
root.title("Dogzilla Control Panel")
root.geometry("700x600")

lbl_img = tk.Label(root)
lbl_img.pack(pady=10)

frame_buttons = tk.Frame(root)
frame_buttons.pack()

btn_forward = tk.Button(frame_buttons, text="Forward ↑", width=10,
                        command=lambda: send_command("forward"))
btn_backward = tk.Button(frame_buttons, text="Backward ↓", width=10,
                         command=lambda: send_command("back"))  # Use "back" here
btn_left = tk.Button(frame_buttons, text="Left ←", width=10,
                     command=lambda: send_command("left"))
btn_right = tk.Button(frame_buttons, text="Right →", width=10,
                      command=lambda: send_command("right"))
btn_stop = tk.Button(root, text="STOP", width=20, bg="red", fg="white",
                     command=lambda: send_command("stop"))

btn_forward.grid(row=0, column=1, padx=5, pady=5)
btn_left.grid(row=1, column=0, padx=5, pady=5)
btn_backward.grid(row=1, column=1, padx=5, pady=5)
btn_right.grid(row=1, column=2, padx=5, pady=5)
btn_stop.pack(pady=10)

# --- START CAMERA THREAD ---
running = True
threading.Thread(target=update_frame, daemon=True).start()

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
