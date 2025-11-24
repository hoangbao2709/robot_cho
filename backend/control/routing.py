# control/routing.py
from django.urls import re_path
from .consumers import ControlInputConsumer

websocket_urlpatterns = [
    # FE sẽ connect: ws://127.0.0.1:8000/ws/robots/robot-a/control/
    re_path(
        r"ws/robots/(?P<robot_id>[^/]+)/control/$",
        ControlInputConsumer.as_asgi(),
    ),
]
