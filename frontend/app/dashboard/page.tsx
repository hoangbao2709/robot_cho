"use client";

import { useMemo, useState } from "react";
import ConnectionCard from "@/components/ConnectionCard";
import { useRouter } from "next/navigation";

type Device = {
  id: number;
  name: string;
  ip: string;
  battery: number;
  status: "online" | "offline" | "unknown";
};

const API_BASE =
  process.env.NEXT_PUBLIC_API_BASE || "http://127.0.0.1:8000/control";
const robotId = "robot-a";

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
  const [devices, setDevices] = useState<Device[]>([
    {
      id: 1,
      name: "Robot A",
      ip: "192.168.2.100",
      battery: 100,
      status: "online",
    },
    {
      id: 2,
      name: "Robot B",
      ip: "192.168.2.101",
      battery: 90,
      status: "offline",
    },
  ]);
  const [addr, setAddr] = useState("");
  const [errorMsg, setErrorMsg] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const router = useRouter();

  const canAdd = useMemo(() => addr.trim().length > 0, [addr]);

  // Add một robot mới vào list (chưa connect)
  const handleAdd = async () => {
    const ip = addr.trim();
    if (!ip || loading) return;

    setErrorMsg(null);

    // Nếu đã tồn tại device với IP này thì không thêm nữa
    if (devices.some((d) => d.ip === ip)) {
      setAddr("");
      return;
    }

    const nextId = devices.length
      ? Math.max(...devices.map((d) => d.id)) + 1
      : 1;
    const nextDevice: Device = {
      id: nextId,
      name: `Robot ${String.fromCharCode(64 + devices.length + 1)}`, // Robot C, D, ...
      ip,
      battery: 100,
      status: "unknown",
    };

    setDevices((prev) => [nextDevice, ...prev]);
    setAddr("");
  };

  // Khi bấm nút Connect trên từng card
  const handleConnectDevice = async (device: Device) => {
    if (loading) return;

    setErrorMsg(null);
    setLoading(true);

    // Chuẩn hoá IP → control & lidar URLs
    const { control, lidar } = extractAddresses(device.ip);

    try {
      const res = await connectRobot(control);
      const status: Device["status"] = res.connected ? "online" : "offline";

      // cập nhật status trong list
      setDevices((prev) =>
        prev.map((d) =>
          d.id === device.id ? { ...d, status } : d
        )
      );

      if (!res.connected) {
        setErrorMsg(res.error || "Không kết nối được tới robot.");
        return;
      }

      // ⚠️ Truyền đầy đủ 2 địa chỉ qua query để FE sử dụng
      router.push(
        `/control?ip=${encodeURIComponent(control)}&lidar=${encodeURIComponent(lidar)}`
      );

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

  function extractAddresses(ipOrUrl: string) {
    let raw = ipOrUrl.trim();

    // Nếu user nhập "http://192.168.1.26:9000"
    if (raw.startsWith("http://") || raw.startsWith("https://")) {
      try {
        const u = new URL(raw);
        const host = u.hostname;
        return {
          ip: host,
          control: `http://${host}:9000`,
          lidar: `http://${host}:8080`,
        };
      } catch {
        // nếu URL lỗi, fallback
      }
    }

    // Nếu user nhập "192.168.1.26"
    return {
      ip: raw,
      control: `http://${raw}:9000`,
      lidar: `http://${raw}:8080`,
    };
  }


  return (
    <section className="p-6">
      <h1 className="gradient-title mb-6">Connection Manager</h1>

      <div className="mb-6 flex gap-2">
        <input
          type="text"
          placeholder="Enter device IP or address"
          value={addr}
          onChange={(e) => setAddr(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && canAdd && handleAdd()}
          className="flex-1 rounded-xl bg-white/10 px-4 py-2 text-sm placeholder:text-white/60 focus:outline-none focus:ring-2 focus:ring-pink-400/60"
        />
        <button
          onClick={handleAdd}
          disabled={!canAdd || loading}
          className={`gradient-button1 px-4 py-2 rounded-xl cursor-pointer text-sm font-medium ${
            (!canAdd || loading) ? "opacity-50 cursor-not-allowed" : ""
          }`}
        >
          {loading ? "Connecting..." : "Add"}
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
          />
        ))}
      </div>
    </section>
  );
}
