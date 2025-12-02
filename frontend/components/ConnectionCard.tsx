"use client";

import { useEffect, useState } from "react";

type Device = {
  id: number;
  name: string;
  ip: string;
};

type CardStatus = {
  status: "online" | "offline" | "unknown";
  battery: number | null;
};

const API_BASE =
  process.env.NEXT_PUBLIC_API_BASE || "http://127.0.0.1:8000/control";
const robotId = "robot-a";

export default function ConnectionCard({
  device,
  onConnect,
  onDelete,
}: {
  device: Device;
  onConnect?: (device: Device) => void;
  onDelete?: (device: Device) => void;
}) {
  const [info, setInfo] = useState<CardStatus>({
    status: "unknown",
    battery: null,
  });

  // Lấy pin + trạng thái (chỉ cần biết gọi được là online)
  useEffect(() => {
    let alive = true;

    async function fetchStatus() {
      try {
        const res = await fetch(
          `${API_BASE}/api/robots/${robotId}/status/`,
          { cache: "no-store" }
        );

        if (!alive) return;
        if (!res.ok) throw new Error("Bad status");

        const data = await res.json();

        // CỨ gọi API thành công là coi như online
        setInfo({
          status: "online",
          battery: data.battery ?? null,
        });
      } catch {
        if (!alive) return;

        setInfo({
          status: "offline",
          battery: null,
        });
      }
    }

    fetchStatus();
    const timer = setInterval(fetchStatus, 2000);

    return () => {
      alive = false;
      clearInterval(timer);
    };
  }, []);

  const statusClass =
    info.status === "online"
      ? "bg-green-500/20 text-green-300"
      : info.status === "offline"
      ? "bg-rose-500/20 text-rose-300"
      : "bg-yellow-500/20 text-yellow-300";

  const batteryText =
    info.battery != null ? `${info.battery}%` : info.status === "offline" ? "-" : "…";

  return (
    <div className="flex justify-between items-center rounded-2xl bg-white/10 p-4">
      {/* Left info */}
      <div>
        <div className="font-semibold flex items-center gap-2">
          <span>{device.name}</span>
          <span className={`text-xs px-2 py-0.5 rounded-full ${statusClass}`}>
            {info.status}
          </span>
        </div>

        <div className="text-sm text-white/70">IP: {device.ip}</div>
        <div className="text-sm text-white/70">Battery: {batteryText}</div>
      </div>

      {/* Right buttons */}
      <div className="flex flex-col gap-2 items-end">
        <button
          onClick={() => onConnect && onConnect(device)}
          className="rounded-xl bg-gradient-to-r from-pink-500 to-purple-500 px-4 py-2 text-sm cursor-pointer w-24 text-center"
        >
          Connect
        </button>

        {onDelete && (
          <button
            onClick={() => onDelete(device)}
            className="rounded-xl bg-red-500/20 text-red-300 px-4 py-1 text-xs cursor-pointer w-24 text-center hover:bg-red-500/30"
          >
            Delete
          </button>
        )}
      </div>
    </div>
  );
}
