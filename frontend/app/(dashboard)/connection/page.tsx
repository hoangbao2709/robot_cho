"use client";
import { useState } from "react";
import GradientButton from "@/components/GradientButton";
import ConnectionCard from "@/components/ConnectionCard";

export default function ConnectionPage() {
  const [devices, setDevices] = useState([
    { id: 1, name: "Robot A", ip: "192.168.2.100", battery: 100, status: "online" },
    { id: 2, name: "Robot B", ip: "192.168.2.101", battery: 90, status: "offline" },
  ]);
  const [addr, setAddr] = useState("");

  return (
    <section>
      <h1 className="mb-6 text-2xl font-bold">
        <span className="text-pink-400">Connection</span> Manager
      </h1>

      <div className="flex gap-2 mb-6">
        <input
          type="text"
          placeholder="Enter device address"
          value={addr}
          onChange={(e) => setAddr(e.target.value)}
          className="flex-1 rounded-xl bg-white/10 px-4 py-2 text-sm placeholder:text-white/60 focus:outline-none"
        />
        <GradientButton onClick={() => setAddr("")}>Add</GradientButton>
      </div>

      <div className="grid gap-4">
        {devices.map((dev) => (
          <ConnectionCard key={dev.id} device={dev} />
        ))}
      </div>
    </section>
  );
}
