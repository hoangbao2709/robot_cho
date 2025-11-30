"use client";

import { useState, useEffect, useRef, useCallback, useMemo } from "react";
import { useSearchParams } from "next/navigation";
import { HalfCircleJoystick } from "@/components/HalfCircleJoystick";
import HeaderControl from "@/components/header_control";
const API_BASE =
  process.env.NEXT_PUBLIC_API_BASE || "http://127.0.0.1:8000/control";
const DEFAULT_DOG_SERVER =
  process.env.NEXT_PUBLIC_DOGZILLA_BASE || "http://127.0.0.1:9000";


const LIDAR_VIEW_URL =
  process.env.NEXT_PUBLIC_DOGZILLA_BASE || "http://127.0.0.1:9000";
const robotId = "robot-a";

async function api<T = any>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${API_BASE}${path}`, {
    ...init,
    headers: {
      "Content-Type": "application/json",
      ...(init?.headers || {}),
    },
    cache: "no-store",
  });
  if (!res.ok) throw new Error(await res.text());
  return res.json();
}

const RobotAPI = {
  connect: (addr: string) =>
    api(`/api/robots/${robotId}/connect/`, {
      method: "POST",
      body: JSON.stringify({ addr }),
    }),
  status: () => api(`/api/robots/${robotId}/status/`),
  fpv: () => api(`/api/robots/${robotId}/fpv/`),
  speed: (mode: "slow" | "normal" | "high") =>
    api(`/api/robots/${robotId}/command/speed/`, {
      method: "POST",
      body: JSON.stringify({ mode }),
    }),
  move: (cmd: {
    vx: number;
    vy: number;
    vz: number;
    rx: number;
    ry: number;
    rz: number;
  }) =>
    api(`/api/robots/${robotId}/command/move/`, {
      method: "POST",
      body: JSON.stringify(cmd),
    }),
  lidar: (action: "start" | "stop") =>
    api(`/api/robots/${robotId}/command/lidar/`, {
      method: "POST",
      body: JSON.stringify({ action }),
    }),
  posture: (name: string) =>
    api(`/api/robots/${robotId}/command/posture/`, {
      method: "POST",
      body: JSON.stringify({ name }),
    }),
  behavior: (name: string) =>
    api(`/api/robots/${robotId}/command/behavior/`, {
      method: "POST",
      body: JSON.stringify({ name }),
    }),
  body: (sl: {
    tx: number;
    ty: number;
    tz: number;
    rx: number;
    ry: number;
    rz: number;
  }) =>
    api(`/api/robots/${robotId}/command/body_adjust/`, {
      method: "POST",
      body: JSON.stringify(sl),
    }),
  // ====== NEW: ổn định / cân bằng ======
  stabilizingMode: (action: "on" | "off" | "toggle") =>
    api(`/api/robots/${robotId}/command/stabilizing_mode/`, {
      method: "POST",
      body: JSON.stringify({ action }),
    }),
};

export default function RemoteView({
  onEmergencyStop,
  mode,
  toggleMode
}: {
  onEmergencyStop?: () => void;
  mode: "remote" | "fpv";
  toggleMode: () => void;
}) {
  const searchParams = useSearchParams();
  const ipParam = searchParams.get("ip"); // /control?ip=...
  const DOG_SERVER = ipParam || DEFAULT_DOG_SERVER;
  // Lidar viewer URL: cùng IP với DOG_SERVER nhưng port 8080
  const lidarUrl = useMemo(() => {
    try {
      const url = new URL(DOG_SERVER);
      url.port = "8080";          // đổi port
      return url.toString();      // ví dụ: http://192.168.1.167:8080/
    } catch {
      // fallback nếu DOG_SERVER không phải URL hợp lệ
      return "http://127.0.0.1:8080";
    }
  }, [DOG_SERVER]);

  const [speed, setSpeed] = useState<"slow" | "normal" | "high">("normal");
  const [fps, setFps] = useState(30);
  const [streamUrl, setStreamUrl] = useState<string | null>(null);
  const [sliders, setSliders] = useState({
    tx: 0,
    ty: 0,
    tz: 0,
    rx: 0,
    ry: 0,
    rz: 0,
  });
  const [isRunning, setIsRunning] = useState(false);

  const [connected, setConnected] = useState(false);
  const [connectError, setConnectError] = useState<string | null>(null);

  // NEW: trạng thái cân bằng
  const [stabilizing, setStabilizing] = useState(false);

  const postureBtns = ["Lie_Down", "Stand_Up", "Sit_Down", "Squat", "Crawl"];
  const axisMotionBtns = [
    "Turn_Roll",
    "Turn_Pitch",
    "Turn_Yaw",
    "3_Axis",
    "Turn_Around",
  ];
  const behavior1 = ["Wave_Hand", "Handshake", "Pray", "Stretch", "Swing"];
  const behavior2 = ["Wave_Body", "Handshake", "Pee", "Play_Ball", "Mark_Time"];

  // ====== Kết nối Django -> Dogzilla Flask server + lấy stream_url ======
  useEffect(() => {
    let stop = false;

    (async () => {
      try {
        // 1) Gọi connect để set robot.addr trong Django
        const res = await RobotAPI.connect(DOG_SERVER);
        if (stop) return;

        if (res?.connected) {
          setConnected(true);
          setConnectError(null);

          // 2) Sau khi connect ok, gọi fpv để lấy stream_url
          try {
            const f = await RobotAPI.fpv();
            if (!stop) setStreamUrl(f?.stream_url || null);
          } catch (e) {
            console.error("FPV error:", e);
            if (!stop)
              setConnectError("Không lấy được stream_url từ backend");
          }
        } else {
          setConnected(false);
          setConnectError(
            res?.error || "Không kết nối được tới Dogzilla server"
          );
        }
      } catch (e: any) {
        console.error("Connect error:", e);
        if (!stop) {
          setConnected(false);
          setConnectError(e?.message || "Lỗi kết nối");
        }
      }
    })();

    // Poll status định kỳ (nếu sau này cần đọc fps/battery thì dùng)
    const iv = setInterval(async () => {
      try {
        // const s = await RobotAPI.status();
        // if (!stop && typeof s?.fps === "number") setFps(s.fps);
      } catch {
        /* ignore */
      }
    }, 2000);

    return () => {
      stop = true;
      clearInterval(iv);
      onEmergencyStop?.();
    };
  }, [DOG_SERVER, onEmergencyStop]);

  const changeSpeed = useCallback(async (m: "slow" | "normal" | "high") => {
    setSpeed(m);
    try {
      await RobotAPI.speed(m);
    } catch (e) {
      console.error("Speed error:", e);
    }
  }, []);

  /* ====== LIDAR TOGGLE ====== */
  const handleToggleLidar = useCallback(async () => {
    const next = !isRunning;
    setIsRunning(next);
    try {
      await RobotAPI.lidar(next ? "start" : "stop");
    } catch (e) {
      console.error("Lidar error:", e);
    }
  }, [isRunning]);

  /* ====== STABILIZING TOGGLE ====== */
  const handleToggleStabilizing = useCallback(async () => {
    const next = !stabilizing;
    setStabilizing(next);
    try {
      // nếu muốn dùng "toggle" thì đổi lại action: "toggle"
      await RobotAPI.stabilizingMode(next ? "on" : "off");
    } catch (e) {
      console.error("Stabilizing error:", e);
      // nếu lỗi thì rollback UI state
      setStabilizing((prev) => !prev);
    }
  }, [stabilizing]);

  const joyRef = useRef({ vx: 0, rz: 0, active: false });
  const maxV = 0.4;
  const maxW = 1.2;

  const sendMoveTick = useCallback(async () => {
    const { vx, rz, active } = joyRef.current;
    if (!active) return;
    try {
      await RobotAPI.move({ vx, vy: 0, vz: 0, rx: 0, ry: 0, rz });
    } catch {
      // im lặng, tránh spam console quá nhiều
    }
  }, []);

  useEffect(() => {
    const iv = setInterval(sendMoveTick, 66);
    return () => clearInterval(iv);
  }, [sendMoveTick]);

  const onJoyChange = useCallback(
    ({ angleDeg, power }: { angleDeg: number; power: number }) => {
      const rad = (angleDeg * Math.PI) / 180;
      const vx = Math.cos(rad) * power * maxV;
      const rz = Math.sin(rad) * power * maxW;
      joyRef.current = { vx, rz, active: true };
    },
    []
  );

  const onJoyRelease = useCallback(async () => {
    joyRef.current = { vx: 0, rz: 0, active: false };
    try {
      await RobotAPI.move({
        vx: 0,
        vy: 0,
        vz: 0,
        rx: 0,
        ry: 0,
        rz: 0,
      });
    } catch {
      /* ignore */
    }
  }, []);

  const turnLeft = () =>
    RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: +0.8 });
  const turnRight = () =>
    RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: -0.8 });
  const stopMove = () =>
    RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: 0 });

  const bodyTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const setBody = (next: typeof sliders) => {
    setSliders(next);
    if (bodyTimer.current) clearTimeout(bodyTimer.current);
    bodyTimer.current = setTimeout(() => {
      RobotAPI.body(next).catch(() => {
        /* ignore */
      });
    }, 150);
  };

  return (
    <section className="min-h-screen w-full bg-[#0c0520] text-white p-6">
      {/* Hiển thị trạng thái connect */}

      <div className="mt-2 text-sm">

        <span className={connected ? "text-emerald-400" : "text-rose-400"}>
          {connected ? "Connected" : "Not connected"}
        </span>
        <span className="ml-2 text-xs opacity-70">Dogzilla: {DOG_SERVER}</span>
        {connectError && (
          <div className="text-xs text-rose-400 mt-1">
            Error: {connectError}
          </div>
        )}
      </div>

      {/* Main content row */}
      <div className="mt-6 grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* FPV video */}
        <div className="lg:col-span-2">
          <HeaderControl mode={mode} onToggle={toggleMode} />

          <div className="relative overflow-hidden rounded-2xl border border-white/10 bg-black">
            <div className="absolute left-3 top-2 text-green-300 text-xl font-bold drop-shadow">
              FPS:{fps}
            </div>
            <img
              src={streamUrl || "/placeholder.svg?height=360&width=640"}
              alt="FPV"
              className="w-full aspect-video object-cover opacity-80"
            />
          </div>
          {/* Bottom control grids */}
          <div className="mt-8 grid grid-cols-1 lg:grid-cols-2 gap-6">
            {/* Basic Postures */}
            <Panel title="Basic Postures">
              <div className="grid grid-cols-2 sm:grid-cols-5 gap-3">
                {postureBtns.map((b) => (
                  <Btn
                    key={b}
                    label={b.replaceAll("_", " ")}
                    onClick={() => RobotAPI.posture(b)}
                  />
                ))}
              </div>
            </Panel>

            <Panel title="Axis Motion">
              <div className="grid grid-cols-2 sm:grid-cols-5 gap-3">
                {axisMotionBtns.map((b) => (
                  <Btn
                    key={b}
                    label={b.replaceAll("_", " ")}
                    onClick={() => RobotAPI.behavior(b)}
                  />
                ))}
              </div>
            </Panel>
          </div>

          {/* Behavior control */}
          <div className="mt-6">
            <Panel title="Behavior Control">
              <div className="grid grid-cols-2 sm:grid-cols-10 gap-3">
                {[...behavior1, ...behavior2].map((b, i) => (
                  <Btn
                    key={`${b}-${i}`}
                    label={b.replaceAll("_", " ")}
                    onClick={() => RobotAPI.behavior(b)}
                  />
                ))}
              </div>
            </Panel>
          </div>
        </div>



        {/* Right control panel */}
        <div className="space-y-6">
          {/* Lidar map */}
          <div className="p-4 rounded-2xl bg-white/5 border border-white/10">
            <div className="text-sm mb-2 opacity-80">Lidar map</div>

            <div
              className="
                relative
                w-full
                max-w-md
                aspect-square
                rounded-xl
                overflow-hidden
                border border-white/10
                bg-black
              "
            >
              <div
                className="
                  absolute inset-0
                  origin-top-left
                  scale-[0.6]       /* GIẢM TỶ LỆ 35% → chỉnh theo ý bạn */
                "
                style={{ width: "600%", height: "600%" }}
              >
                <iframe
                  src={lidarUrl}
                  className="w-full h-full border-0"
                />
              </div>
            </div>
          </div>




          {/* Speed */}
          <Panel title="Speed">
            <div className="flex gap-3">
              {(["slow", "normal", "high"] as const).map((s) => (
                <Chip
                  key={s}
                  label={s.charAt(0).toUpperCase() + s.slice(1)}
                  active={speed === s}
                  onClick={() => changeSpeed(s)}
                />
              ))}
            </div>
          </Panel>

          <Panel title="Move">
            <div className="grid grid-cols-1 sm:grid-cols-[auto_1fr] gap-4 items-start">
              {/* Joystick */}
              <div className="justify-self-center sm:justify-self-start cursor-pointer">
                <HalfCircleJoystick
                  width={260}
                  height={160}
                  rest="center"
                  onChange={onJoyChange}
                  onRelease={onJoyRelease}
                />
              </div>

              <div className="grid grid-cols-2 gap-2">
                <Btn label="Turn left" onClick={turnLeft} />
                <Btn label="Turn right" onClick={turnRight} />
                <Btn variant="danger" label="Stop" onClick={stopMove} />
                <Btn
                  variant={isRunning ? "danger" : "success"}
                  label={isRunning ? "Stop Lidar" : "Start Lidar"}
                  onClick={handleToggleLidar}
                />
                {/* NEW: nút bật/tắt cân bằng */}
                <Btn
                  variant={stabilizing ? "success" : "default"}
                  label={stabilizing ? "Stabilizing ON" : "Stabilizing OFF"}
                  onClick={handleToggleStabilizing}
                />
              </div>
            </div>
          </Panel>

          {/* Body Adjustment */}
          <Panel title="Body Adjustment">
            <SliderRow
              label="Translation_X"
              value={sliders.tx}
              onChange={(v) => setBody({ ...sliders, tx: v })}
            />
            <SliderRow
              label="Translation_Y"
              value={sliders.ty}
              onChange={(v) => setBody({ ...sliders, ty: v })}
            />
            <SliderRow
              label="Translation_Z"
              value={sliders.tz}
              onChange={(v) => setBody({ ...sliders, tz: v })}
            />
            <SliderRow
              label="Rotation_X"
              value={sliders.rx}
              onChange={(v) => setBody({ ...sliders, rx: v })}
            />
            <SliderRow
              label="Rotation_Y"
              value={sliders.ry}
              onChange={(v) => setBody({ ...sliders, ry: v })}
            />
            <SliderRow
              label="Rotation_Z"
              value={sliders.rz}
              onChange={(v) => setBody({ ...sliders, rz: v })}
            />
          </Panel>

        </div>
      </div>


    </section>
  );
}

function Panel({
  title,
  children,
}: {
  title: string;
  children: React.ReactNode;
}) {
  return (
    <div className="rounded-2xl bg-white/5 border border-white/10 p-4">
      <div className="text-sm mb-3 opacity-80">{title}</div>
      {children}
    </div>
  );
}

function Chip({
  label,
  active,
  onClick,
}: {
  label: string;
  active?: boolean;
  onClick?: () => void;
}) {
  return (
    <button
      onClick={onClick}
      className={`px-4 py-1 rounded-xl text-sm border transition cursor-pointer
      ${active
          ? "bg-indigo-500/30 border-indigo-400/40"
          : "bg-white/5 border-white/10 hover:bg-white/10"
        }`}
    >
      {label}
    </button>
  );
}

function Btn({
  label,
  variant = "default",
  onClick,
}: {
  label: string;
  variant?: "default" | "danger" | "success";
  onClick?: () => void;
}) {
  const base =
    "px-4 py-2 text-sm rounded-xl border font-medium transition-all duration-200 cursor-pointer transform";

  let styles = "";

  if (variant === "danger") {
    styles =
      "bg-rose-600/70 border-rose-400 text-white " +
      "hover:bg-rose-400 hover:border-rose-200 hover:text-white " +
      "hover:shadow-lg hover:shadow-rose-500/40 hover:scale-105 active:scale-95";
  } else if (variant === "success") {
    styles =
      "bg-emerald-600/70 border-emerald-400 text-white " +
      "hover:bg-emerald-400 hover:border-emerald-200 hover:text-white " +
      "hover:shadow-lg hover:shadow-emerald-500/40 hover:scale-105 active:scale-95";
  } else {
    // default
    styles =
      "bg-white/10 border-white/30 text-white " +
      "hover:bg-white hover:text-[#0c0520] hover:border-white " +
      "hover:shadow-lg hover:shadow-white/40 hover:scale-105 active:scale-95";
  }

  return (
    <button onClick={onClick} className={`${base} ${styles}`}>
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
    <div className="mb-3">
      <div className="text-xs mb-1 opacity-75">{label}</div>
      <input
        type="range"
        min={-100}
        max={100}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        className="w-full accent-fuchsia-400 cursor-pointer"
      />
    </div>
  );
}
