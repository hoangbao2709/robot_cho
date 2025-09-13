# robot_client/camera.py
# -*- coding: utf-8 -*-
import threading
import time
import requests
import cv2
import numpy as np

class _OpenCVCapture:
    def __init__(self, url):
        self.url = url
        self.cap = None

    def open(self):
        self.cap = cv2.VideoCapture(self.url)
        if not self.cap.isOpened():
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = cv2.VideoCapture(self.url)
        return self.cap.isOpened()

    def read(self):
        if self.cap is None:
            return False, None
        return self.cap.read()

    def release(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass


class _HTTPMjpegCapture:
    """
    Fallback đọc MJPEG multipart/x-mixed-replace qua HTTP.
    Server dùng boundary=frame và Content-Type: image/jpeg.
    """
    def __init__(self, url, timeout=5, chunk_size=4096):
        self.url = url
        self.timeout = timeout
        self.chunk_size = chunk_size
        self._resp = None
        self._iter = None
        self._buf = bytearray()
        self.boundary = b"--frame"
        self.ct_jpeg = b"Content-Type: image/jpeg"

    def open(self):
        try:
            self._resp = requests.get(self.url, stream=True, timeout=self.timeout)
            self._resp.raise_for_status()
            self._iter = self._resp.iter_content(chunk_size=self.chunk_size)
            return True
        except Exception as e:
            print("[HTTPMjpeg] open error:", e)
            self._resp = None
            self._iter = None
            return False

    def _next_jpeg_bytes(self):
        if self._iter is None:
            return None
        for chunk in self._iter:
            if not chunk:
                continue
            self._buf.extend(chunk)
            while True:
                i0 = self._buf.find(self.boundary)
                if i0 < 0:
                    break
                if i0 > 0:
                    del self._buf[:i0]
                ih = self._buf.find(self.ct_jpeg)
                if ih < 0:
                    break
                he = self._buf.find(b"\r\n\r\n", ih)
                if he < 0:
                    break
                data_start = he + 4
                nb = self._buf.find(self.boundary, data_start)
                if nb < 0:
                    break
                jpg = bytes(self._buf[data_start:nb-2])
                del self._buf[:nb]
                return jpg
        return None

    def read(self):
        jpg = self._next_jpeg_bytes()
        if jpg is None:
            return False, None
        arr = np.frombuffer(jpg, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return False, None
        return True, frame

    def release(self):
        try:
            if self._resp is not None:
                self._resp.close()
        except Exception:
            pass


class Camera:
    """
    Camera reader:
      - Thử OpenCV VideoCapture với HTTP trước.
      - Nếu fail, fallback sang HTTP MJPEG (requests).
    Dùng:
        cam = Camera("http://<pi>:9000/camera")
        cam.start()
        frame = cam.get_latest()
    """
    def __init__(self, url, retry_sec=2.0):
        self.url = url
        self.retry_sec = retry_sec
        self._frame = None
        self._lock = threading.Lock()
        self._running = False
        self._t = None
        self.backend = None  

    def start(self):
        if self._t and self._t.is_alive():
            return
        self._running = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def stop(self):
        self._running = False

    def get_latest(self):
        with self._lock:
            return self._frame

    def _loop(self):
        while self._running:
            # 1) thử OpenCV
            cvcap = _OpenCVCapture(self.url)
            if cvcap.open():
                print("[CameraClient] Using OpenCV VideoCapture")
                self.backend = "_cv"
                while self._running:
                    ok, frame = cvcap.read()
                    if not ok or frame is None:
                        break
                    with self._lock:
                        self._frame = frame
                cvcap.release()
                print("[CameraClient] OpenCV stream ended, retrying in %.1fs..." % self.retry_sec)
                time.sleep(self.retry_sec)
                continue

            # 2) fallback HTTP
            httpcap = _HTTPMjpegCapture(self.url, timeout=5)
            if httpcap.open():
                print("[CameraClient] Using HTTP MJPEG fallback (requests)")
                self.backend = "_http"
                while self._running:
                    ok, frame = httpcap.read()
                    if not ok or frame is None:
                        break
                    with self._lock:
                        self._frame = frame
                httpcap.release()
                print("[CameraClient] HTTP MJPEG ended, retrying in %.1fs..." % self.retry_sec)
                time.sleep(self.retry_sec)
                continue

            print("[CameraClient] Cannot open stream, retry in %.1fs..." % self.retry_sec)
            time.sleep(self.retry_sec)

class CameraReader(Camera):
    def __init__(self, url=None):
        if url is None:
            try:
                from . import config
                url = config.CAMERA_URL
            except Exception:
                raise RuntimeError("CameraReader needs URL or config.CAMERA_URL")
        super().__init__(url)
