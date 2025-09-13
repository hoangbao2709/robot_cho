# -*- coding: utf-8 -*-
import tkinter as tk
from .control import Control
from .camera import Camera
from .mouselook import MouseLook
from .ui import AppUI
from . import config

def main():
    root = tk.Tk()

    control = Control()

    camera = Camera(config.CAMERA_URL)
    camera.start()

    mouselook = MouseLook(root, control)
    app = AppUI(root, control, camera, mouselook)

    try:
        control.stop()
    except Exception:
        pass

    def on_closing():
        try:
            control.stop()
        except Exception:
            pass
        try:
            camera.stop()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
