"use client";
import { useMemo, useState } from "react";
import GradientButton from "@/components/GradientButton";
import ConnectionCard from "@/components/ConnectionCard";

type Device = {
  id: number;
  name: string;
  ip: string;
  battery: number;     
  status: "online" | "offline" | "unknown";
};

export default function DashboardPage() {
  const [devices, setDevices] = useState<Device[]>([
    { id: 1, name: "Robot A", ip: "192.168.2.100", battery: 100, status: "online" },
    { id: 2, name: "Robot B", ip: "192.168.2.101", battery: 90, status: "offline" },
  ]);
  const [addr, setAddr] = useState("");

  const canAdd = useMemo(() => addr.trim().length > 0, [addr]);

  const handleAdd = () => {
    const ip = addr.trim();
    if (!ip) return;

    if (devices.some(d => d.ip === ip)) {
      setAddr("");
      return;
    }

    const next: Device = {
      id: devices.length ? Math.max(...devices.map(d => d.id)) + 1 : 1,
      name: `Robot ${String.fromCharCode(64 + devices.length + 1)}`, 
      ip,
      battery: 100,
      status: "unknown",
    };
    setDevices(prev => [next, ...prev]);
    setAddr("");
  };

  return (
    <section className="p-6">
      <h1 className="mb-6 text-2xl font-bold">
        <span className="text-pink-400">Connection</span> Manager
      </h1>

      <div className="mb-6 flex gap-2">
        <input
          type="text"
          placeholder="Enter device IP or address"
          value={addr}
          onChange={(e) => setAddr(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && canAdd && handleAdd()}
          className="flex-1 rounded-xl bg-white/10 px-4 py-2 text-sm placeholder:text-white/60 focus:outline-none focus:ring-2 focus:ring-pink-400/60"
        />
        <GradientButton
          onClick={handleAdd}
          disabled={!canAdd}
        >
          Add
        </GradientButton>
      </div>

      <div className="grid gap-4 sm:grid-cols-2 xl:grid-cols-3">
        {devices.map((dev) => (
          <ConnectionCard key={dev.id} device={dev} />
        ))}
      </div>
    </section>
  );
}
