"use client";

import React, { useMemo, useState, useEffect } from "react";
import ConnectionCard from "@/components/ConnectionCard";
import { useRouter } from "next/navigation";

export type Device = {
  id: number;
  name: string;
  ip: string; 
  battery: number;
  url?: string;
  status: "online" | "offline" | "unknown";
  source?: "manual" | "cloudflare";
};

const BACKEND_BASE =
  process.env.NEXT_PUBLIC_BACKEND_BASE || "http://127.0.0.1:8000";

// Base cho các API điều khiển robot (Django app "control")
const CONTROL_BASE = `${BACKEND_BASE}/control`;

const robotId = "robot-a";
const DEVICES_COOKIE_KEY = "dogzilla_devices";

// ========================
// Helpers cookie
// ========================
function saveDevicesToCookie(devices: Device[]) {
  if (typeof document === "undefined") return;
  try {
    // chỉ lưu device do user tự nhập (manual)
    const manualDevices = devices.filter((d) => d.source !== "cloudflare");
    const raw = JSON.stringify(manualDevices);
    document.cookie = `${DEVICES_COOKIE_KEY}=${encodeURIComponent(
      raw
    )}; path=/; max-age=31536000`;
  } catch (err) {
    console.error("Cannot save devices to cookie:", err);
  }
}

function loadDevicesFromCookie(): Device[] | null {
  if (typeof document === "undefined") return null;
  try {
    const cookies = document.cookie.split(";").map((c) => c.trim());
    const found = cookies.find((c) => c.startsWith(`${DEVICES_COOKIE_KEY}=`));
    if (!found) return null;

    const value = decodeURIComponent(found.split("=")[1] || "");
    if (!value) return null;

    const parsed = JSON.parse(value);
    return parsed as Device[];
  } catch (err) {
    console.error("Cannot parse devices cookie:", err);
    return null;
  }
}

// ========================
// Gọi API connect trên Django
// ========================
async function connectRobot(addr: string) {
  const res = await fetch(
    `${CONTROL_BASE}/api/robots/${robotId}/connect/`,
    {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ addr }),
      cache: "no-store",
    }
  );

  if (!res.ok) {
    throw new Error(await res.text());
  }

  return res.json() as Promise<{ connected: boolean; error?: string }>;
}

// ========================
// Component chính
// ========================
export default function DashboardPage() {
  const [devices, setDevices] = useState<Device[]>([]);
  const [addr, setAddr] = useState("");
  const [errorMsg, setErrorMsg] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const router = useRouter();

  const canAdd = useMemo(() => addr.trim().length > 0, [addr]);

  // 1) Load manual devices từ cookie
  useEffect(() => {
    const stored = loadDevicesFromCookie();
    if (stored && stored.length > 0) {
      setDevices(stored);
    }
  }, []);

  // 2) Hỏi backend xem robot_url mới nhất là gì (Cloudflare)
  useEffect(() => {
    if (typeof window === "undefined") return;

    async function fetchProfile() {
      try {
        // lấy token nếu có (nếu chưa làm login thì token có thể null, vẫn chạy bình thường)
        const token = localStorage.getItem("access_token");
        const headers: HeadersInit = {};
        if (token) {
          headers["Authorization"] = `Bearer ${token}`;
        }

        const res = await fetch(`${BACKEND_BASE}/api/auth/me/`, {
          headers,
        });

        console.log("[me] status =", res.status);

        const json = await res.json().catch(() => null);
        console.log("[me] json =", json);

        if (!res.ok || !json) {
          // Nếu token sai / hết hạn -> xoá token luôn
          if (res.status === 401 && json?.code === "token_not_valid") {
            localStorage.removeItem("access_token");
            localStorage.removeItem("refresh_token");
          }
          console.warn("[me] backend returned error, skip cloudflare card");
          return;
        }

        const robotUrl = (json.robot_url as string | null) ?? null;
        const robotDeviceId =
          (json.robot_device_id as string | null) ?? "rpi5-dogzilla";

        if (!robotUrl) {
          console.log("[me] No robot_url for this user -> no Cloudflare card");
          return;
        }

        // lưu vào localStorage để chỗ khác dùng nếu cần
        localStorage.setItem("robot_url", robotUrl);
        localStorage.setItem("robot_device_id", robotDeviceId);

        // cập nhật / tạo card Cloudflare trong danh sách devices
        setDevices((prev) => {
          const cfId = 0;
          const exists = prev.find((d) => d.id === cfId);

          const cfDevice: Device = {
            id: cfId,
            name: "My Robot (Cloudflare)",
            ip: robotUrl, // full URL: https://xxx.trycloudflare.com
            battery: exists?.battery ?? 100,
            status: exists?.status ?? "unknown",
            source: "cloudflare",
          };

          if (exists) {
            // update card cũ
            return prev.map((d) => (d.id === cfId ? cfDevice : d));
          }
          // luôn đưa Cloudflare card lên đầu
          return [cfDevice, ...prev];
        });
      } catch (err) {
        console.error("fetchProfile error:", err);
      }
    }

    fetchProfile();
  }, []);

  // 3) Mỗi khi devices thay đổi -> lưu lại cookie (chỉ manual)
  useEffect(() => {
    saveDevicesToCookie(devices);
  }, [devices]);

  // 4) Connect khi bấm trên card
  const handleConnectDevice = async (device: Device) => {
    if (loading) return;

    setErrorMsg(null);
    setLoading(true);

    let dogzillaAddr = device.ip.trim();

    // Nếu không phải URL đầy đủ thì coi là IP nội bộ -> thêm http + port 9000
    if (
      !dogzillaAddr.startsWith("http://") &&
      !dogzillaAddr.startsWith("https://")
    ) {
      dogzillaAddr = `http://${dogzillaAddr}:9000`;
    }

    try {
      const res = await connectRobot(dogzillaAddr);
      const status: Device["status"] = res.connected ? "online" : "offline";

      setDevices((prev) =>
        prev.map((d) => (d.id === device.id ? { ...d, status } : d))
      );

      if (!res.connected) {
        setErrorMsg(res.error || "Không kết nối được tới robot.");
        return;
      }

      router.push(`/control?ip=${encodeURIComponent(dogzillaAddr)}`);
    } catch (e: any) {
      console.error("Connect error:", e);
      setDevices((prev) =>
        prev.map((d) =>
          d.id === device.id ? { ...d, status: "offline" } : d
        )
      );
      setErrorMsg(e?.message || "Không kết nối được tới backend");
    } finally {
      setLoading(false);
    }
  };

  const handleDeleteDevice = (device: Device) => {
    setDevices((prev) => prev.filter((d) => d.id !== device.id));
  };

  const handleAdd = () => {
    const ip = addr.trim();
    if (!ip || loading) return;

    setErrorMsg(null);

    if (devices.some((d) => d.ip === ip)) {
      setAddr("");
      return;
    }

    const nextId = devices.length
      ? Math.max(...devices.map((d) => d.id)) + 1
      : 1;

    const nextDevice: Device = {
      id: nextId,
      name: `Robot ${String.fromCharCode(64 + devices.length + 1)}`, // Robot A/B/C...
      ip,
      battery: 100,
      status: "unknown",
      source: "manual",
    };

    setDevices((prev) => [nextDevice, ...prev]);
    setAddr("");
  };

  return (
    <section className="p-6">
      <h1 className="gradient-title mb-6">Connection Manager</h1>

      <div className="mb-6 flex gap-2">
        <input
          type="text"
          placeholder="Enter device IP (vd: 192.168.2.100)"
          value={addr}
          onChange={(e) => setAddr(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && canAdd && handleAdd()}
          className="flex-1 rounded-xl bg-white/10 px-4 py-2 text-sm placeholder:text-white/60 focus:outline-none focus:ring-2 focus:ring-pink-400/60"
        />
        <button
          onClick={handleAdd}
          disabled={!canAdd || loading}
          className={`gradient-button1 px-4 py-2 rounded-xl cursor-pointer text-sm font-medium ${
            !canAdd || loading ? "opacity-50 cursor-not-allowed" : ""
          }`}
        >
          Add
        </button>
      </div>

      {errorMsg && (
        <div className="mb-4 text-xs text-rose-400">Error: {errorMsg}</div>
      )}

      <div className="grid gap-4 sm:grid-cols-2 xl:grid-cols-3">
        {devices.map((dev) => (
          <ConnectionCard
            key={dev.id}
            device={dev}
            onConnect={handleConnectDevice}
            onDelete={handleDeleteDevice}
          />
        ))}
      </div>
    </section>
  );
}
