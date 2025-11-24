"use client";

import { useState, useEffect, useRef, useCallback, useMemo } from "react";
import { useSearchParams } from "next/navigation";
import { HalfCircleJoystick } from "@/components/HalfCircleJoystick";
import header_control from "@/components/header_control";
import MouselookPad from "@/components/MouselookPad";
import { RobotAPI, DEFAULT_DOG_SERVER, robotId } from "../lib/robotApi";

export default function RemoteControlMode() {
  const searchParams = useSearchParams();
  const ipParamRaw = searchParams.get("ip") || "";
  const lidarParam = searchParams.get("lidar");

  // ===== Chuẩn hoá ip -> DOG_SERVER (9000) & LIDAR_URL (8080) =====
  const { DOG_SERVER, LIDAR_URL } = useMemo(() => {
    // Nếu đã truyền sẵn ?lidar=... từ trang connection thì ưu tiên dùng luôn
    if (lidarParam) {
      return {
        DOG_SERVER: ipParamRaw || DEFAULT_DOG_SERVER,
        LIDAR_URL: lidarParam,
      };
    }

    // Không có ip -> fallback default
    if (!ipParamRaw) {
      return {
        DOG_SERVER: DEFAULT_DOG_SERVER,
        LIDAR_URL:
          process.env.NEXT_PUBLIC_LIDAR_URL || "http://127.0.0.1:8080",
      };
    }

    let host = ipParamRaw.trim();

    // Nếu user truyền full URL thì tách hostname ra
    try {
      if (host.startsWith("http://") || host.startsWith("https://")) {
        const u = new URL(host);
        host = u.hostname;
      }
    } catch {
      // nếu parse URL lỗi thì giữ nguyên (coi như ip thuần)
    }

    return {
      DOG_SERVER: `http://${host}:9000`,
      LIDAR_URL: `http://${host}:8080`,
    };
  }, [ipParamRaw, lidarParam]);

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

  const [isRunning, setIsRunning] = useState(false); // Lidar
  const [connected, setConnected] = useState(false);
  const [connectError, setConnectError] = useState<string | null>(null);

  const [mouseLook, setMouseLook] = useState(false);

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

  /* ================= KẾT NỐI BACKEND + FPV ================= */
  useEffect(() => {
    let stop = false;

    (async () => {
      try {
        const res = await RobotAPI.connect(DOG_SERVER);
        if (stop) return;

        if (res?.connected) {
          setConnected(true);
          setConnectError(null);

          try {
            const f = await RobotAPI.fpv();
            if (!stop) setStreamUrl(f?.stream_url || null);
          } catch (e) {
            console.error("FPV error:", e);
            if (!stop) {
              setConnectError("Không lấy được stream_url từ backend");
            }
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

    // (Tuỳ chọn) đọc status định kỳ: FPS, battery...
    const iv = setInterval(async () => {
      try {
        // const s = await RobotAPI.status();
        // if (!stop && typeof s?.fps === "number") setFps(s.fps);
      } catch { }
    }, 2000);

    return () => {
      stop = true;
      clearInterval(iv);
    };
  }, [DOG_SERVER]);

  /* ================= SPEED / LIDAR ================= */

  const changeSpeed = useCallback(async (m: "slow" | "normal" | "high") => {
    setSpeed(m);
    try {
      await RobotAPI.speed(m);
    } catch (e) {
      console.error("Speed error:", e);
    }
  }, []);

  const handleToggleLidar = useCallback(async () => {
    const next = !isRunning;
    setIsRunning(next);
    try {
      await RobotAPI.lidar(next ? "start" : "stop");
    } catch (e) {
      console.error("Lidar error:", e);
    }
  }, [isRunning]);

  /* ================= JOYSTICK -> /command/move ================= */

  const joyRef = useRef({ vx: 0, rz: 0, active: false });
  const maxV = 0.4; // m/s
  const maxW = 1.2; // rad/s

  const sendMoveTick = useCallback(async () => {
    const { vx, rz, active } = joyRef.current;
    if (!active) return;
    try {
      await RobotAPI.move({ vx, vy: 0, vz: 0, rx: 0, ry: 0, rz });
    } catch {
      // im lặng
    }
  }, []);

  useEffect(() => {
    const iv = setInterval(sendMoveTick, 66); // ~15 Hz
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
    } catch { }
  }, []);

  const turnLeft = () =>
    RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: +0.8 });
  const turnRight = () =>
    RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: -0.8 });
  const stopMove = () =>
    RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: 0 });

  /* ================= BODY SLIDERS (debounce) ================= */

  const bodyTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const setBody = (next: typeof sliders) => {
    setSliders(next);
    if (bodyTimer.current) clearTimeout(bodyTimer.current);
    bodyTimer.current = setTimeout(() => {
      RobotAPI.body(next).catch(() => { });
    }, 150);
  };

  /* ====================================================== */

  function Metric({ label, value }: { label: string; value: string }) {
    return (
      <div className="rounded-xl bg-white/5 border border-white/10 p-3">
        <div className="text-[10px] uppercase opacity-60">{label}</div>
        <div className="text-sm mt-1">{value}</div>
      </div>
    );
  }

  return (
    <section className="min-h-screen w-full bg-[#0c0520] text-white p-6">
      {header_control()}

      {/* Trạng thái connect */}
      <div className="mt-2 text-sm">
        <span className={connected ? "text-emerald-400" : "text-rose-400"}>
          {connected ? "Connected" : "Not connected"}
        </span>
        <span className="ml-2 text-xs opacity-70">Dogzilla: {DOG_SERVER}</span>
        {connectError && (
          <div className="text-xs text-rose-400 mt-1">Error: {connectError}</div>
        )}
      </div>

      {/* Hàng chính: Robot info + FPV + panel bên phải */}
      <div className="mt-6 grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Cột trái: Robot details + FPV + behaviors */}
        <div className="lg:col-span-2">
          {/* Robot info */}
          <div className="col-span-2 p-4 rounded-2xl bg-white/5 border border-white/10">
            <div className="flex flex-wrap items-center gap-6">
              <div className="text-lg font-semibold">🤖 Robot A</div>
              <div className="text-xs uppercase opacity-60">Robot Details</div>
            </div>
            <div className="mt-4 grid grid-cols-3 gap-4 text-sm">
              <Metric label="Location" value="25.23234, 19.76543" />
              <Metric label="Cleaning Progress" value={`80% (stopped)`} />
              <Metric label="Floor" value="1st" />
              <Metric label="Status" value="Resting" />
              <Metric label="Water Level" value="50%" />
              <Metric label="Battery" value="85%" />
            </div>
          </div>

          {/* FPV */}
          <div className="mt-4 relative overflow-hidden rounded-2xl border border-white/10 bg-black">
            <div className="absolute left-3 top-2 text-green-300 text-xl font-bold drop-shadow">
              FPS:{fps}
            </div>
            <img
              src={streamUrl || "/placeholder.svg?height=360&width=640"}
              alt="FPV"
              className="w-full aspect-video object-cover opacity-80"
            />
          </div>

          {/* Postures & Axis Motion */}
          <div className="mt-8 grid grid-cols-1 lg:grid-cols-2 gap-6">
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

          {/* Behaviors */}
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

        {/* Cột phải: Lidar + Speed + Move + MouseLook + Body */}
        <div className="space-y-6">
          {/* Lidar map */}
          <div className="p-4 rounded-2xl bg-white/5 border border-white/10">
            <div className="text-sm mb-2 opacity-80">Lidar map</div>
            <div className="relative pt-[100%] bg-black rounded-xl overflow-hidden">
              <iframe
                src={LIDAR_URL}
                className="absolute inset-0 w-full h-full border-0"
                loading="lazy"
              />
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

          {/* Move */}
          <Panel title="Move">
            <div className="grid grid-cols-1 sm:grid-cols-[auto_1fr] gap-4 items-start">
              <div className="justify-self-center sm:justify-self-start cursor-pointer">
                <HalfCircleJoystick
                  width={260}
                  height={160}
                  rest="center"
                  onChange={onJoyChange}
                  onRelease={onJoyRelease}
                />
              </div>
              <div>
                <div className="grid grid-cols-2 gap-2">
                  <Btn label="Turn left" onClick={turnLeft} />
                  <Btn label="Turn right" onClick={turnRight} />
                  <Btn variant="danger" label="Stop" onClick={stopMove} />
                  <Btn
                    variant={isRunning ? "danger" : "success"}
                    label={isRunning ? "Stop Lidar" : "Start Lidar"}
                    onClick={handleToggleLidar}
                  />
                </div>
                <div className="flex justify-between pt-7 items-center">
                  <div className="text-xl opacity-70">
                    Toggle để bật/tắt điều khiển bằng chuột.
                  </div>
                  <MouseLookToggle
                    on={mouseLook}
                    onToggle={() => setMouseLook((prev) => !prev)}
                  />
                </div>
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

      {/* Overlay điều khiển chuột + WASD */}
      <MouselookPad robotId={robotId} enabled={mouseLook} />
    </section>
  );
}

/* ================= UI HELPERS ================= */

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

function MouseLookToggle({
  on,
  onToggle,
}: {
  on: boolean;
  onToggle: () => void;
}) {
  return (
    <button
      onClick={onToggle}
      className={`w-11 h-7 rounded-full border border-violet-400/50 p-1 grid items-center transition ${on ? "bg-violet-500/30" : "bg-transparent"
        }`}
      aria-label="Toggle MouseLook"
    >
      <div
        className={`w-5 h-5 rounded-full border border-violet-300/60 bg-white/10 transition-transform ${on ? "translate-x-4" : ""
          }`}
      />
    </button>
  );
}




