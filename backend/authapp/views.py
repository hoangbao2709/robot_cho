from django.contrib.auth.models import User
from django.contrib.auth import authenticate
from rest_framework.decorators import api_view, permission_classes
from rest_framework.permissions import IsAuthenticated, AllowAny   # <---

from rest_framework.response import Response
from rest_framework import status
from rest_framework_simplejwt.tokens import RefreshToken
from django.conf import settings
from .mongo import users_collection
from rest_framework_simplejwt.authentication import JWTAuthentication
from rest_framework.exceptions import AuthenticationFailed
from datetime import datetime, timezone   # <--- c?n cho save_user_url



@api_view(["POST"])
def register_view(request):
    username = request.data.get("username")
    email = request.data.get("email")
    password = request.data.get("password")

    if not username or not email or not password:
        return Response(
            {"error": "Username, email and password are required"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    if len(password) < 6:
        return Response(
            {"error": "Password must be at least 6 characters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    if User.objects.filter(username=username).exists():
        return Response(
            {"error": "Username already taken"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    if User.objects.filter(email=email).exists():
        return Response(
            {"error": "Email already registered"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # tạo user trong SQLite (Django auth)
    user = User.objects.create_user(
        username=username,
        email=email,
        password=password,
    )

    # sync sang Mongo
    users_collection.update_one(
        {"email": user.email},
        {
            "$set": {
                "email": user.email,
                "username": user.username,
                "is_superuser": user.is_superuser,
                "is_staff": user.is_staff,
            }
        },
        upsert=True,
    )

    # trả luôn JWT để FE auto login sau khi đăng ký
    refresh = RefreshToken.for_user(user)

    return Response(
        {
            "ok": True,
            "message": "Registered successfully",
            "email": user.email,
            "username": user.username,
            "access": str(refresh.access_token),
            "refresh": str(refresh),
        },
        status=status.HTTP_201_CREATED,
    )


@api_view(["POST"])
def login_view(request):
    email = request.data.get("email")
    password = request.data.get("password")

    if not email or not password:
        return Response(
            {"error": "Email and password are required"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    try:
        user_obj = User.objects.get(email=email)
        username = user_obj.username
    except User.DoesNotExist:
        return Response(
            {"error": "Invalid credentials"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    user = authenticate(username=username, password=password)
    if user is None:
        return Response(
            {"error": "Invalid credentials"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # sync sang Mongo mỗi lần login (nếu có thay đổi)
    users_collection.update_one(
        {"email": user.email},
        {
            "$set": {
                "email": user.email,
                "username": user.username,
                "is_superuser": user.is_superuser,
                "is_staff": user.is_staff,
            }
        },
        upsert=True,
    )
    
    mongo_doc = users_collection.find_one({"email": user.email}) or {}
    robot_url = mongo_doc.get("robot_url")
    robot_device_id = mongo_doc.get("robot_device_id")

    refresh = RefreshToken.for_user(user)

    return Response(
        {
            "ok": True,
            "email": user.email,
            "username": user.username,
            "access": str(refresh.access_token),
            "refresh": str(refresh),
            "robot_url": robot_url,
            "robot_device_id": robot_device_id,
        },
        status=status.HTTP_200_OK,
    )
    
@api_view(["POST"])
@permission_classes([IsAuthenticated])
def save_user_url(request):
    url = request.data.get("url")

    if not url:
        return Response(
            {"error": "url is required"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    user = request.user
    now = datetime.now(timezone.utc)

    users_collection.update_one(
        {"email": user.email},
        {
            "$set": {
                "url": url,
                "updated_at": now,
            }
        },
        upsert=True, 
    )

    return Response({"ok": True, "message": "URL saved"}, status=status.HTTP_200_OK)

@api_view(["POST"])
@permission_classes([AllowAny])   # CHO PHÉP KHÔNG CẦN JWT
def link_robot(request):
    """
    Robot gọi endpoint này mỗi lần boot để cập nhật URL Cloudflare.
    Bảo vệ bằng ROBOT_REG_SECRET.
    """
    # 1. Check secret từ header
    secret = request.headers.get("X-Robot-Secret")
    if secret != settings.ROBOT_REG_SECRET:
        return Response(
            {"detail": "Forbidden"},
            status=status.HTTP_403_FORBIDDEN,
        )

    email = request.data.get("email")
    url = request.data.get("url")
    device_id = request.data.get("device_id")

    if not email or not url or not device_id:
        return Response(
            {"detail": "email, url, device_id are required"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    from .mongo import users_collection  # hoặc chỗ nào bạn connect Mongo

    # Ví dụ: update user theo email
    users_collection.update_one(
        {"email": email},
        {
            "$set": {
                "robot_url": url,
                "robot_device_id": device_id,
            }
        },
        upsert=False,
    )

    return Response({"ok": True}, status=status.HTTP_200_OK)

@api_view(["GET"])
@permission_classes([AllowAny])   # tạm thời AllowAny, ta tự check JWT bên trong
def me_view(request):
    """
    Trả về user + robot_url, nhưng tự xác thực JWT để dễ debug.
    """

    jwt_auth = JWTAuthentication()
    try:
        auth_result = jwt_auth.authenticate(request)
    except AuthenticationFailed as e:
        # In ra chi tiết lỗi JWT trong console
        print("JWTAuthentication failed:", e.detail)
        return Response(
            {"detail": e.detail},
            status=status.HTTP_401_UNAUTHORIZED,
        )

    if auth_result is None:
        # Không có header Authorization hoặc format sai
        print("JWTAuthentication: no credentials found in request")
        return Response(
            {"detail": "No authentication credentials found"},
            status=status.HTTP_401_UNAUTHORIZED,
        )

    user, validated_token = auth_result
    print("JWT ok for user:", user.username)

    # Phần logic cũ
    doc = users_collection.find_one({"email": user.email}) or {}

    return Response(
        {
            "ok": True,
            "email": user.email,
            "username": user.username,
            "robot_url": doc.get("robot_url"),
            "robot_device_id": doc.get("robot_device_id"),
            "robot_updated_at": doc.get("robot_updated_at"),
        },
        status=status.HTTP_200_OK,
    )