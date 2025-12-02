"use client";

import { useMemo, useState, useEffect } from "react";
import ConnectionCard from "@/components/ConnectionCard";
import { useRouter } from "next/navigation";

export type Device = {
  id: number;
  name: string;
  ip: string;
  battery: number;
  status: "online" | "offline" | "unknown";
};

const API_BASE =
  process.env.NEXT_PUBLIC_API_BASE || "http://127.0.0.1:8000/control";
const robotId = "robot-a";

const DEVICES_COOKIE_KEY = "dogzilla_devices";

// ========================
// Helpers cookie
// ========================
function saveDevicesToCookie(devices: Device[]) {
  try {
    const raw = JSON.stringify(devices);
    document.cookie = `${DEVICES_COOKIE_KEY}=${encodeURIComponent(
      raw
    )}; path=/; max-age=31536000`; // ~1 năm
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

// Hàm gọi API connect trên Django
async function connectRobot(addr: string) {
  const res = await fetch(`${API_BASE}/api/robots/${robotId}/connect/`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ addr }),
    cache: "no-store",
  });
  if (!res.ok) {
    throw new Error(await res.text());
  }
  return res.json() as Promise<{ connected: boolean; error?: string }>;
}

export default function DashboardPage() {
  // Có thể để rỗng, hoặc để default 2 con như ban đầu
  const [devices, setDevices] = useState<Device[]>([]);
  const [addr, setAddr] = useState("");
  const [errorMsg, setErrorMsg] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const router = useRouter();

  const canAdd = useMemo(() => addr.trim().length > 0, [addr]);

  // Lần đầu mount: load devices từ cookie (nếu có)
  useEffect(() => {
    const stored = loadDevicesFromCookie();
    if (stored && stored.length > 0) {
      setDevices(stored);
    }
  }, []);

  // Mỗi khi devices thay đổi -> lưu lại cookie
  useEffect(() => {
    saveDevicesToCookie(devices);
  }, [devices]);

  // Connect khi bấm trên card
  const handleConnectDevice = async (device: Device) => {
    if (loading) return;

    setErrorMsg(null);
    setLoading(true);

    let dogzillaAddr = device.ip.trim();
    if (!dogzillaAddr.startsWith("http")) {
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

  // Xoá robot khỏi list + cookie
  const handleDeleteDevice = (device: Device) => {
    setDevices((prev) => prev.filter((d) => d.id !== device.id));
  };

  // Add robot mới -> chỉ LƯU + HIỆN, KHÔNG auto-connect
  const handleAdd = () => {
    const ip = addr.trim();
    if (!ip || loading) return;

    setErrorMsg(null);

    // Nếu đã tồn tại device với IP này thì chỉ clear input thôi
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
