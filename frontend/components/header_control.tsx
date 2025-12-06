"use client";

import React, { useEffect, useState } from "react";
import Link from "next/link";
type HeaderControlProps = {
  mode: "remote" | "fpv";
  onToggle: () => void;
  lidarUrl?: string | null;
  connected: boolean;
};

type SystemTelemetry = {
  cpu_percent: number | null;
  ram: string | null;
  disk: string | null;
  ip: string | null;
  time: string | null;
};

type Telemetry = {
  robot_connected: boolean;
  turn_speed_range?: [number, number];
  step_default?: number;
  z_range?: [number, number];
  z_current?: number;
  pitch_range?: [number, number];
  pitch_current?: number;
  battery?: number | null;
  fw?: string | null;
  fps?: number | null;
  system?: SystemTelemetry | null;
};

// Pose từ Lidar server
type LidarPose = {
  x: number;
  y: number;
  theta?: number;
  connected: string;
};

const API_BASE =
  process.env.NEXT_PUBLIC_API_BASE || "http://127.0.0.1:8000/control";
const robotId = "robot-a";

async function api<T = any>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${API_BASE}${path}`, {
    ...init,
    headers: { "Content-Type": "application/json", ...(init?.headers || {}) },
    cache: "no-store",
  });
  if (!res.ok) {
    throw new Error(`Request failed: ${res.status}`);
  }
  return res.json();
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-xl bg-white/5 border border-white/10 p-3">
      <div className="text-[10px] uppercase opacity-60">{label}</div>
      <div className="text-sm mt-1">{value}</div>
    </div>
  );
}

function buildLidarPoseUrl(lidarUrl: string) {
  if(!lidarUrl){
    return "";
  }
  if (lidarUrl.endsWith("/pose") || lidarUrl.endsWith("/pose/")) {
    return lidarUrl;
  }

  try {
    const u = new URL(lidarUrl);
    if (u.pathname.endsWith("/pose") || u.pathname.endsWith("/pose/")) {
      return u.toString();
    }
    if (!u.pathname.endsWith("/")) {
      u.pathname += "/";
    }
    u.pathname += "pose";
    return u.toString();
  } catch {

    return `${lidarUrl.replace(/\/$/, "")}/pose`;
  }
}

export default function HeaderControl({
  mode,
  onToggle,
  lidarUrl,
  connected,
}: HeaderControlProps) {
  const isFPV = mode === "fpv";

  const [robotName, setRobotName] = useState<string>("Robot A");
  const [telemetry, setTelemetry] = useState<Telemetry | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  // Text hiển thị LOCATION
  const [locationText, setLocationText] = useState<string>("-");

  // ====== Poll /status/ trên backend ======
  useEffect(() => {
    let isMounted = true;

    async function fetchStatus() {
      try {
        const data = await api<any>(`/api/robots/${robotId}/status/`);
        if (!isMounted) return;

        setRobotName(data.name || "Robot A");

        const t: Telemetry =
          data.telemetry ?? {
            robot_connected: data.robot_connected ?? false,
            turn_speed_range: data.turn_speed_range,
            step_default: data.step_default,
            z_range: data.z_range,
            z_current: data.z_current,
            pitch_range: data.pitch_range,
            pitch_current: data.pitch_current,
            battery: data.battery,
            fw: data.fw,
            fps: data.fps,
            system: data.system ?? null,
          };

        setTelemetry(t);
        setError(null);
        setLoading(false);
      } catch (e: any) {
        console.error("Fetch status error", e);
        if (!isMounted) return;
        setError("Cannot fetch robot status");
        setLoading(false);
      }
    }

    fetchStatus();
    const id = setInterval(fetchStatus, 2000);
    return () => {
      isMounted = false;
      clearInterval(id);
    };
  }, []);

  // ====== Poll Lidar server (lidarUrl) để lấy (x, y, theta) ======
  useEffect(() => {
    if (!lidarUrl) {
      setLocationText("-");
      return;
    }

    const poseUrl = buildLidarPoseUrl(lidarUrl);
    let stop = false;
    
    async function fetchPose() {
      try {
        const res = await fetch(poseUrl, { cache: "no-store" });
        if (!res.ok) throw new Error(`Pose HTTP ${res.status}`);
        const raw = await res.json();

        if (stop) return;

        const p: LidarPose = raw; // server trả { x, y, theta }

        if (typeof p?.x === "number" && typeof p?.y === "number") {
          const x = p.x.toFixed(2);
          const y = p.y.toFixed(2);
          const thetaText =
            typeof p.theta === "number" ? `, θ: ${p.theta.toFixed(2)} rad` : "";
          setLocationText(`x: ${x} m, y: ${y} m${thetaText}`);
        } else {
          setLocationText("-");
        }
      } catch (e) {
        if (!stop) {
          setLocationText("-");
        }
      }
    }

    fetchPose();
    const id = setInterval(fetchPose, 1000);

    return () => {
      stop = true;
      clearInterval(id);
    };
  }, [lidarUrl]);

  const sys: SystemTelemetry | null = telemetry?.system ?? null;

  const cpuText =
    sys?.cpu_percent != null ? `${sys.cpu_percent}%` : loading ? "…" : "-";
  const ramText = sys?.ram ?? (loading ? "…" : "-");
  const diskText = sys?.disk ?? (loading ? "…" : "-");
  const ipText = sys?.ip ?? (loading ? "…" : "-");
  const batteryValue =
    telemetry?.battery != null ? `${telemetry.battery}%` : loading ? "…" : "-";

  return (
    <>
      {/* Header trên cùng */}
      <header className="flex items-center justify-between">
        <h1
          className={`gradient-title select-none transition-all duration-300 ${
            isFPV ? "opacity-100" : "opacity-90"
          }`}
        >
          {isFPV ? "FPV CONTROL MODE" : "REMOTE CONTROL MODE"}
        </h1>

        <div className="flex items-center gap-3">
          <Link
            href="/control"
            className="
              px-3 py-1 rounded-xl
              bg-pink-500/20 
              hover:bg-pink-500/30 
              text-pink-300 text-sm
              transition-all
              hover:scale-[1.03]
              active:scale-95
            "
          >
            Disconnect
          </Link>

          <span className="text-sm opacity-70">Remote</span>
          <button
            onClick={onToggle}
            className={`w-11 h-7 rounded-full border border-violet-400/50 p-1 grid items-center transition-all duration-300 ${
              isFPV ? "bg-violet-500/40" : "bg-transparent"
            }`}
            aria-label="Toggle FPV mode"
          >
            <div
              className={`w-5 h-5 rounded-full border border-violet-300/60 bg-white/10 transition-transform duration-300 ${
                isFPV ? "translate-x-4" : ""
              }`}
            />
          </button>
          <span className="text-sm">FPV</span>
        </div>
      </header>

      {/* Hàng dưới: Robot info + metrics */}
      <div className="mt-4 grid grid-cols-1 lg:grid-cols-2 gap-6">
        <div className="col-span-2 p-4 rounded-2xl bg-white/5 border border-white/10">
          <div className="flex flex-wrap items-center gap-6">
            <div className="text-lg font-semibold">🤖 {robotName}</div>
            {sys?.time && (
              <div className="text-xs opacity-60">Time: {sys.time}</div>
            )}
            {error && <div className="text-xs text-red-400">{error}</div>}
                    <span className={connected ? "text-emerald-400" : "text-rose-400"}>
          {connected ? "Connected" : "Not connected"}
        </span>
          </div>

          <div className="mt-4 grid grid-cols-3 gap-4 text-sm">
            <Metric label="Location" value={locationText} />
            <Metric label="CPU" value={cpuText} />
            <Metric label="RAM" value={ramText} />
            <Metric label="SDC" value={diskText} />
            <Metric label="IPA" value={ipText} />
            <Metric label="Battery" value={batteryValue} />
          </div>
        </div>
      </div>
    </>
  );
}
