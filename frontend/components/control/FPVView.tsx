"use client";

import React, { useEffect, useRef, useState } from "react";

const API_BASE = process.env.NEXT_PUBLIC_API_BASE || "http://127.0.0.1:8000/control";
const robotId = "robot-a";

async function api<T = any>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${API_BASE}${path}`, {
    ...init,
    headers: { "Content-Type": "application/json", ...(init?.headers || {}) },
    cache: "no-store",
  });
  if (!res.ok) throw new Error(await res.text());
  return res.json();
}

const RobotAPI = {
  posture: (name: string) =>
    api(`/api/robots/${robotId}/command/posture/`, { method: "POST", body: JSON.stringify({ name }) }),
  behavior: (name: string) =>
    api(`/api/robots/${robotId}/command/behavior/`, { method: "POST", body: JSON.stringify({ name }) }),
  body: (sl: { tx: number; ty: number; tz: number; rx: number; ry: number; rz: number }) =>
    api(`/api/robots/${robotId}/command/body_adjust/`, { method: "POST", body: JSON.stringify(sl) }),
};

export default function FPVView({
  streamUrl,
  fps = 30,
}: {
  streamUrl?: string;
  fps?: number;
}) {
  const postureBtns = ["Lie_Down", "Stand_Up", "Sit_Down", "Squat", "Crawl"];
  const axisMotionBtns = ["Turn_Roll", "Turn_Pitch", "Turn_Yaw", "3_Axis", "Turn_Around"];
  const behavior1 = ["Wave_Hand", "Handshake", "Pray", "Stretch", "Swing"];
  const behavior2 = ["Wave_Body", "Handshake", "Pee", "Play_Ball", "Mark_Time"];

  const [sliders, setSliders] = useState({ tx: 0, ty: 0, tz: 0, rx: 0, ry: 0, rz: 0 });

  // Debounce gửi body adjust
  const bodyTimer = useRef<NodeJS.Timeout | null>(null);
  const setBody = (next: typeof sliders) => {
    setSliders(next);
    if (bodyTimer.current) clearTimeout(bodyTimer.current);
    bodyTimer.current = setTimeout(() => { RobotAPI.body(next).catch(() => {}); }, 150);
  };
  useEffect(() => () => { if (bodyTimer.current) clearTimeout(bodyTimer.current); }, []);

  return (
    <div className="space-y-6">
      {/* FPV video */}
      <div className="relative overflow-hidden rounded-2xl border border-white/10 bg-black shadow-[0_10px_30px_rgba(0,0,0,0.35)]">
        <div className="absolute left-3 top-2 text-green-300 text-xl font-bold drop-shadow">FPS:{fps}</div>
        <img
          src={streamUrl || "/placeholder.svg?height=360&width=640"}
          alt="FPV"
          className="w-full aspect-[16/7] object-cover"
        />
      </div>

      {/* 2 cột: Trái = sliders ; Phải = buttons */}
      <div className="grid grid-cols-1 xl:grid-cols-2 gap-6 items-start">
        {/* LEFT — Body Adjustment */}
        <Panel title="Body Adjustment">
          <SliderRow label="Translation_X" value={sliders.tx} onChange={(v) => setBody({ ...sliders, tx: v })} />
          <SliderRow label="Translation_Y" value={sliders.ty} onChange={(v) => setBody({ ...sliders, ty: v })} />
          <SliderRow label="Translation_Z" value={sliders.tz} onChange={(v) => setBody({ ...sliders, tz: v })} />
          <SliderRow label="Rotation_X" value={sliders.rx} onChange={(v) => setBody({ ...sliders, rx: v })} />
          <SliderRow label="Rotation_Y" value={sliders.ry} onChange={(v) => setBody({ ...sliders, ry: v })} />
          <SliderRow label="Rotation_Z" value={sliders.rz} onChange={(v) => setBody({ ...sliders, rz: v })} />
        </Panel>

        {/* RIGHT — buttons xếp dọc */}
        <div className="flex flex-col gap-6">
          <Panel title="Basic Postures">
            {/* auto-fit để nút tự giãn, mỗi nút min 120px; chữ không xuống dòng */}
            <div className="grid grid-cols-[repeat(auto-fit,minmax(120px,1fr))] gap-3">
              {postureBtns.map((b) => (
                <Btn key={b} label={b.replaceAll("_", " ")} onClick={() => RobotAPI.posture(b)} />
              ))}
            </div>
          </Panel>

          <Panel title="Axis Motion">
            <div className="grid grid-cols-[repeat(auto-fit,minmax(120px,1fr))] gap-3">
              {axisMotionBtns.map((b) => (
                <Btn key={b} label={b.replaceAll("_", " ")} onClick={() => RobotAPI.behavior(b)} />
              ))}
            </div>
          </Panel>

          <Panel title="Behavior Control">
            {/* nhiều nút => min 140px cho dễ đọc */}
            <div className="grid grid-cols-[repeat(auto-fit,minmax(140px,1fr))] gap-3">
              {[...behavior1, ...behavior2].map((b, i) => (
                <Btn key={`${b}-${i}`} label={b.replaceAll("_", " ")} onClick={() => RobotAPI.behavior(b)} />
              ))}
            </div>
          </Panel>
        </div>
      </div>
    </div>
  );
}

/* ===== UI helpers ===== */
function Panel({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div className="rounded-2xl bg-white/5 border border-white/10 p-5 shadow-[inset_0_1px_0_rgba(255,255,255,0.05)]">
      <div className="text-base font-semibold mb-4 opacity-90">{title}</div>
      {children}
    </div>
  );
}

function Btn({
  label,
  onClick,
}: {
  label: string;
  onClick?: () => void;
}) {
  return (
    <button
      onClick={onClick}
      className="inline-flex items-center justify-center px-4 py-2 rounded-xl border border-white/10 bg-white/5 hover:bg-white/10 transition text-sm font-medium whitespace-nowrap min-w-[120px]"
    >
      {label}
    </button>
  );
}

function SliderRow({
  label,
  value,
  onChange,
}: {
  label: string;
  value: number;
  onChange: (v: number) => void;
}) {
  return (
    <div className="mb-4">
      <div className="text-sm mb-1 opacity-80">{label}</div>
      <input
        type="range"
        min={-100}
        max={100}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        className="w-full accent-fuchsia-400"
      />
    </div>
  );
}
