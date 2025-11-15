"use client";

import React, { useState, useEffect, useRef, useCallback } from "react";
import { HalfCircleJoystick } from "@/components/HalfCircleJoystick";

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
  status: () => api(`/api/robots/${robotId}/status/`),
  fpv: () => api(`/api/robots/${robotId}/fpv/`),
  speed: (mode: "slow" | "normal" | "high") =>
    api(`/api/robots/${robotId}/command/speed/`, { method: "POST", body: JSON.stringify({ mode }) }),
  move: (cmd: { vx: number; vy: number; vz: number; rx: number; ry: number; rz: number }) =>
    api(`/api/robots/${robotId}/command/move/`, { method: "POST", body: JSON.stringify(cmd) }),
  lidar: (action: "start" | "stop") =>
    api(`/api/robots/${robotId}/command/lidar/`, { method: "POST", body: JSON.stringify({ action }) }),
  posture: (name: string) =>
    api(`/api/robots/${robotId}/command/posture/`, { method: "POST", body: JSON.stringify({ name }) }),
  behavior: (name: string) =>
    api(`/api/robots/${robotId}/command/behavior/`, { method: "POST", body: JSON.stringify({ name }) }),
  body: (sl: { tx: number; ty: number; tz: number; rx: number; ry: number; rz: number }) =>
    api(`/api/robots/${robotId}/command/body_adjust/`, { method: "POST", body: JSON.stringify(sl) }),
};

export default function RemoteView({ onEmergencyStop }: { onEmergencyStop?: () => void }) {
  const [speed, setSpeed] = useState<"slow" | "normal" | "high">("normal");
  const [fps, setFps] = useState(30);
  const [streamUrl, setStreamUrl] = useState<string | null>(null);
  const [sliders, setSliders] = useState({ tx: 0, ty: 0, tz: 0, rx: 0, ry: 0, rz: 0 });
  const [isRunning, setIsRunning] = useState(false);

  const postureBtns = ["Lie_Down", "Stand_Up", "Sit_Down", "Squat", "Crawl"];
  const axisMotionBtns = ["Turn_Roll", "Turn_Pitch", "Turn_Yaw", "3_Axis", "Turn_Around"];
  const behavior1 = ["Wave_Hand", "Handshake", "Pray", "Stretch", "Swing"];
  const behavior2 = ["Wave_Body", "Handshake", "Pee", "Play_Ball", "Mark_Time"];

  useEffect(() => {
    let stop = false;
    (async () => {
      try {
        const f = await RobotAPI.fpv();
        if (!stop) setStreamUrl(f?.stream_url || null);
      } catch {}
    })();
    const iv = setInterval(async () => {
      try {
        // const s = await RobotAPI.status(); if (typeof s?.fps === "number") setFps(s.fps);
      } catch {}
    }, 2000);
    return () => {
      stop = true;
      clearInterval(iv);
      onEmergencyStop?.();
    };
  }, [onEmergencyStop]);

  const changeSpeed = useCallback(async (m: "slow" | "normal" | "high") => {
    setSpeed(m);
    try { await RobotAPI.speed(m); } catch (e) { console.error("Speed error:", e); }
  }, []);

  const handleToggleLidar = useCallback(async () => {
    const next = !isRunning; setIsRunning(next);
    try { await RobotAPI.lidar(next ? "start" : "stop"); } catch (e) { console.error("Lidar error:", e); }
  }, [isRunning]);

  // ===== Joystick loop =====
  const joyRef = useRef({ vx: 0, rz: 0, active: false });
  const maxV = 0.4, maxW = 1.2;

  const sendMoveTick = useCallback(async () => {
    const { vx, rz, active } = joyRef.current;
    try { if (active) await RobotAPI.move({ vx, vy: 0, vz: 0, rx: 0, ry: 0, rz }); } catch {}
  }, []);
  useEffect(() => { const iv = setInterval(sendMoveTick, 66); return () => clearInterval(iv); }, [sendMoveTick]);

  const onJoyChange = useCallback(({ angleDeg, power }: { angleDeg: number; power: number }) => {
    const rad = (angleDeg * Math.PI) / 180;
    joyRef.current = { vx: Math.cos(rad) * power * maxV, rz: Math.sin(rad) * power * maxW, active: true };
  }, []);
  const onJoyRelease = useCallback(async () => {
    joyRef.current = { vx: 0, rz: 0, active: false };
    try { await RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: 0 }); } catch {}
  }, []);

  const turnLeft = () => RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: +0.8 });
  const turnRight = () => RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: -0.8 });
  const stopMove = () => RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: 0 });

  // Body sliders debounce
  const bodyTimer = useRef<NodeJS.Timeout | null>(null);
  const setBody = (next: typeof sliders) => {
    setSliders(next);
    if (bodyTimer.current) clearTimeout(bodyTimer.current);
    bodyTimer.current = setTimeout(() => { RobotAPI.body(next).catch(() => {}); }, 150);
  };

  return (
    <section className="min-h-screen w-full bg-[#0c0520] text-white p-6">
      {/* ===== Row 1: Video (L) + Speed/Move (R) ===== */}
      <div className="mt-6 grid grid-cols-1 md:[grid-template-columns:1.6fr_1fr] gap-6 items-start">
        {/* Video */}
        <div className="relative overflow-hidden rounded-2xl border border-white/10 bg-black shadow-[0_10px_30px_rgba(0,0,0,0.35)]">
          <div className="absolute left-3 top-2 text-green-300 text-xl font-bold drop-shadow">FPS:{fps}</div>
          <img
            src={streamUrl || "/placeholder.svg?height=360&width=640"}
            alt="FPV"
            className="w-full aspect-video object-cover"
          />
        </div>

        {/* Right column */}
        <div className="flex flex-col gap-6">
          <Panel title="Speed">
            <div className="flex gap-3">
              {(["slow", "normal", "high"] as const).map((s) => (
                <Chip key={s} label={s[0].toUpperCase() + s.slice(1)} active={speed === s} onClick={() => changeSpeed(s)} />
              ))}
            </div>
          </Panel>

          <Panel title="Move">
            {/* Joystick trên, nút dưới — responsive */}
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4 items-start">
              <div className="justify-self-center md:justify-self-start">
                <HalfCircleJoystick width={260} height={160} rest="center" onChange={onJoyChange} onRelease={onJoyRelease}/>
              </div>
              <div className="grid grid-cols-[repeat(auto-fit,minmax(120px,1fr))] gap-3">
                <Btn label="Turn left"  onClick={turnLeft}/>
                <Btn label="Turn right" onClick={turnRight}/>
                <Btn variant="danger" label="Stop" onClick={stopMove}/>
                <Btn variant={isRunning ? "danger" : "success"} label={isRunning ? "Stop Lidar":"Start Lidar"} onClick={handleToggleLidar}/>
              </div>
            </div>
          </Panel>
        </div>
      </div>

      {/* ===== Row 2: Sliders (L) + Posture/Axis/Behavior (R) ===== */}
      <div className="mt-8 grid grid-cols-1 md:[grid-template-columns:1.4fr_1fr] gap-6 items-start">
        {/* Left: Body Adjustment */}
        <Panel title="Body Adjustment">
          <SliderRow label="Translation_X" value={sliders.tx} onChange={(v) => setBody({ ...sliders, tx: v })}/>
          <SliderRow label="Translation_Y" value={sliders.ty} onChange={(v) => setBody({ ...sliders, ty: v })}/>
          <SliderRow label="Translation_Z" value={sliders.tz} onChange={(v) => setBody({ ...sliders, tz: v })}/>
          <SliderRow label="Rotation_X"    value={sliders.rx} onChange={(v) => setBody({ ...sliders, rx: v })}/>
          <SliderRow label="Rotation_Y"    value={sliders.ry} onChange={(v) => setBody({ ...sliders, ry: v })}/>
          <SliderRow label="Rotation_Z"    value={sliders.rz} onChange={(v) => setBody({ ...sliders, rz: v })}/>
        </Panel>

        {/* Right: buttons stack */}
        <div className="flex flex-col gap-6">
          <Panel title="Basic Postures">
            <div className="grid grid-cols-[repeat(auto-fit,minmax(120px,1fr))] gap-3">
              {postureBtns.map((b) => <Btn key={b} label={b.replaceAll("_"," ")} onClick={() => RobotAPI.posture(b)}/>)}
            </div>
          </Panel>

          <Panel title="Axis Motion">
            <div className="grid grid-cols-[repeat(auto-fit,minmax(120px,1fr))] gap-3">
              {axisMotionBtns.map((b) => <Btn key={b} label={b.replaceAll("_"," ")} onClick={() => RobotAPI.behavior(b)}/>)}
            </div>
          </Panel>

          <Panel title="Behavior Control">
            <div className="grid grid-cols-[repeat(auto-fit,minmax(140px,1fr))] gap-3">
              {[...behavior1, ...behavior2].map((b, i) =>
                <Btn key={`${b}-${i}`} label={b.replaceAll("_"," ")} onClick={() => RobotAPI.behavior(b)}/>
              )}
            </div>
          </Panel>
        </div>
      </div>
    </section>
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

function Chip({ label, active, onClick }: { label: string; active?: boolean; onClick?: () => void }) {
  return (
    <button
      onClick={onClick}
      className={`px-4 py-2 rounded-xl text-sm border transition whitespace-nowrap min-w-[100px]
      ${active ? "bg-indigo-500/30 border-indigo-400/40" : "bg-white/5 border-white/10 hover:bg-white/10"}`}
    >
      {label}
    </button>
  );
}

function Btn({ label, variant = "default", onClick }: { label: string; variant?: "default"|"danger"|"success"; onClick?: () => void; }) {
  const base = "inline-flex items-center justify-center px-4 py-2 text-sm rounded-xl border transition whitespace-nowrap min-w-[120px]";
  const styles =
    variant === "danger" ? "bg-rose-500/20 border-rose-400/30 hover:bg-rose-500/30" :
    variant === "success" ? "bg-emerald-500/20 border-emerald-400/30 hover:bg-emerald-500/30" :
    "bg-white/5 border-white/10 hover:bg-white/10";
  return <button onClick={onClick} className={`${base} ${styles}`}>{label}</button>;
}

function SliderRow({ label, value, onChange }: { label: string; value: number; onChange: (v: number) => void }) {
  return (
    <div className="mb-4">
      <div className="text-sm mb-1 opacity-80">{label}</div>
      <input type="range" min={-100} max={100} value={value} onChange={(e) => onChange(Number(e.target.value))} className="w-full accent-fuchsia-400"/>
    </div>
  );
}
