# backend/asgi.py
import os

from django.core.asgi import get_asgi_application
from channels.routing import ProtocolTypeRouter, URLRouter
from channels.auth import AuthMiddlewareStack

from control.routing import websocket_urlpatterns   # <-- lấy WS routes

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "backend.settings")

django_asgi_app = get_asgi_application()

# ASGI application dùng cho cả HTTP + WebSocket
application = ProtocolTypeRouter(
    {
        "http": django_asgi_app,
        "websocket": AuthMiddlewareStack(
            URLRouter(websocket_urlpatterns)
        ),
    }
)
