"use client";

import React, { useEffect, useRef, useState } from "react";
import { useSearchParams } from "next/navigation";

const API_BASE =
  process.env.NEXT_PUBLIC_API_BASE || "http://127.0.0.1:8000/control";
const DEFAULT_DOG_SERVER =
  process.env.NEXT_PUBLIC_DOGZILLA_BASE || "http://127.0.0.1:9000";
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

/* ===== GAMEPAD ===== */

type GamepadSnapshot = {
  connected: boolean;
  axes: number[];
  buttons: boolean[];  
};

const DEADZONE = 0.2;
const SEND_INTERVAL = 80; // ms
const MAX_V = 0.4; // m/s
const MAX_W = 1.2; // rad/s

export default function FPVView({
  fps = 30,
  onEmergencyStop,
}: {
  fps?: number;
  onEmergencyStop?: () => void;
}) {
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
  const [stabilizingOn, setStabilizingOn] = useState(false);
  const [streamUrl, setStreamUrl] = useState<string | null>(null);
  const [sliders, setSliders] = useState({
    tx: 0,
    ty: 0,
    tz: 0,
    rx: 0,
    ry: 0,
    rz: 0,
  });

  const [connected, setConnected] = useState(false);
  const [connectError, setConnectError] = useState<string | null>(null);
  const [lidarRunning, setLidarRunning] = useState(false);
  const lastButtonsRef = useRef<boolean[]>([]);
  const searchParams = useSearchParams();
  const ipParam = searchParams.get("ip");
  const DOG_SERVER = ipParam || DEFAULT_DOG_SERVER;

  /* ===== BODY ADJUST (debounce) ===== */

  const bodyTimer = useRef<ReturnType<typeof setTimeout> | null>(null);

  const setBody = (next: typeof sliders) => {
    setSliders(next);
    if (bodyTimer.current) clearTimeout(bodyTimer.current);
    bodyTimer.current = setTimeout(() => {
      RobotAPI.body(next).catch(() => {});
    }, 150);
  };

  useEffect(
    () => () => {
      if (bodyTimer.current) clearTimeout(bodyTimer.current);
    },
    []
  );

  /* ===== CONNECT + FPV ===== */

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
            if (!stop) setConnectError("Không lấy được stream_url từ backend");
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

    const iv = setInterval(async () => {
      try {
        // const s = await RobotAPI.status();
        // if (!stop && typeof s?.fps === "number") setFps(s.fps);
      } catch {}
    }, 2000);

    return () => {
      stop = true;
      clearInterval(iv);
      onEmergencyStop?.();
    };
  }, [DOG_SERVER, onEmergencyStop]);

  /* ===== GAMEPAD READ LOOP ===== */

  const padRef = useRef<GamepadSnapshot>({
    connected: false,
    axes: [],
    buttons: [],
  });

  const lastMoveRef = useRef<{ vx: number; vy: number; rz: number }>({
    vx: 0,
    vy: 0,
    rz: 0,
  });
  const loggedMappingRef = useRef(false);

  useEffect(() => {
    if (typeof window === "undefined") return;
    const nav: any = navigator;
    if (!nav.getGamepads) {
      console.warn("Browser không hỗ trợ Gamepad API");
      return;
    }

    let rafId: number | null = null;

    const loop = () => {
      const pads = nav.getGamepads() as (Gamepad | null)[];
      const gp = pads[0];

      if (gp) {
        padRef.current = {
          connected: true,
          axes: gp.axes.slice(),
          buttons: gp.buttons.map((b) => b.pressed),   // <-- thêm dòng này
        };

        if (!loggedMappingRef.current) {
          console.log("[Gamepad] axes sample:", gp.axes);
          console.log("[Gamepad] buttons sample:", gp.buttons.map(b => b.pressed));
          loggedMappingRef.current = true;
        }
      } else {
        padRef.current = { connected: false, axes: [], buttons: [] };
      }

      rafId = requestAnimationFrame(loop);
    };


    const handleConnect = (e: GamepadEvent) => {
      console.log("Gamepad connected:", e.gamepad.id);
      if (rafId == null) {
        rafId = requestAnimationFrame(loop);
      }
    };

    const handleDisconnect = (e: GamepadEvent) => {
      console.log("Gamepad disconnected:", e.gamepad.id);
      padRef.current = { connected: false, axes: [], buttons: [] };
    };


    window.addEventListener("gamepadconnected", handleConnect);
    window.addEventListener("gamepaddisconnected", handleDisconnect);

    // nếu tay cầm đã cắm sẵn
    const pads = nav.getGamepads() as (Gamepad | null)[];
    if (pads[0]) {
      rafId = requestAnimationFrame(loop);
    }

    return () => {
      window.removeEventListener("gamepadconnected", handleConnect);
      window.removeEventListener("gamepaddisconnected", handleDisconnect);
      if (rafId != null) cancelAnimationFrame(rafId);
    };
  }, []);

  /* ===== GAMEPAD → MOVE (2 JOYSTICKS) ===== */

useEffect(() => {
  const timer = setInterval(() => {
    const snap = padRef.current;
    if (!snap.connected) return;

    const axes = snap.axes;
    const buttons = snap.buttons;
    const lastButtons = lastButtonsRef.current;

    // ===== LEFT STICK: move =====
    // Axis 0 = left X (strafe), Axis 1 = left Y (forward/back)
    const axLX = axes[0] ?? 0;
    const axLY = axes[1] ?? 0;

    let fwd = -axLY;      // lên = tiến
    let strafe = axLX;    // trái / phải

    if (Math.abs(fwd) < DEADZONE) fwd = 0;
    if (Math.abs(strafe) < DEADZONE) strafe = 0;

    const vx = fwd * MAX_V;
    const vy = strafe * MAX_V;

    // ===== B1 / B3: rotate =====
    const btnB1 = buttons[1] ?? false; // xoay trái
    const btnB3 = buttons[3] ?? false; // xoay phải
    let yaw = 0;
    if (btnB1 && !btnB3) yaw = +1;
    else if (btnB3 && !btnB1) yaw = -1;

    const rz = yaw * MAX_W;

    // ===== gửi lệnh move nếu thay đổi =====
    const last = lastMoveRef.current;
    if (
      Math.abs(vx - last.vx) > 0.01 ||
      Math.abs(vy - last.vy) > 0.01 ||
      Math.abs(rz - last.rz) > 0.01
    ) {
      lastMoveRef.current = { vx, vy, rz };
      RobotAPI.move({ vx, vy, vz: 0, rx: 0, ry: 0, rz }).catch(() => {});
    }

    // ===== B0: toggle LiDAR =====
    const btnB0 = buttons[0] ?? false;
    const prevB0 = lastButtons[0] ?? false;
    if (btnB0 && !prevB0) {
      // vừa nhấn xuống
      const next = !lidarRunning;
      setLidarRunning(next);
      RobotAPI.lidar(next ? "start" : "stop").catch(() => {});
    }

    // ===== B2: toggle stabilizing_mode =====
    const btnB2 = buttons[2] ?? false;
    const prevB2 = lastButtons[2] ?? false;
    if (btnB2 && !prevB2) {
      // vừa nhấn xuống
      const nextStab = !stabilizingOn;
      setStabilizingOn(nextStab);
      // dùng action "toggle" như bạn đã định nghĩa
      RobotAPI.stabilizingMode("toggle").catch(() => {});
      // hoặc nếu muốn sync chặt:
      // RobotAPI.stabilizingMode(nextStab ? "on" : "off").catch(() => {});
    }

    // cập nhật lastButtons cho lần sau
    lastButtonsRef.current = buttons.slice();
  }, SEND_INTERVAL);

  return () => {
    clearInterval(timer);
    // dừng robot khi rời trang
    RobotAPI.move({
      vx: 0,
      vy: 0,
      vz: 0,
      rx: 0,
      ry: 0,
      rz: 0,
    }).catch(() => {});
  };
}, [lidarRunning, stabilizingOn]);



  /* ===== NÚT MANUAL: STRAFE & ROTATE & LIDAR ===== */

  const STRAFE_V = 0.25;
  const TURN_W = 0.8;

  const strafeLeft = () => {
    RobotAPI.move({
      vx: 0,
      vy: +STRAFE_V,
      vz: 0,
      rx: 0,
      ry: 0,
      rz: 0,
    }).catch(() => {});
  };

  const strafeRight = () => {
    RobotAPI.move({
      vx: 0,
      vy: -STRAFE_V,
      vz: 0,
      rx: 0,
      ry: 0,
      rz: 0,
    }).catch(() => {});
  };

  const turnLeft = () => {
    RobotAPI.move({
      vx: 0,
      vy: 0,
      vz: 0,
      rx: 0,
      ry: 0,
      rz: +TURN_W,
    }).catch(() => {});
  };

  const turnRight = () => {
    RobotAPI.move({
      vx: 0,
      vy: 0,
      vz: 0,
      rx: 0,
      ry: 0,
      rz: -TURN_W,
    }).catch(() => {});
  };

  const toggleLidar = () => {
    const next = !lidarRunning;
    setLidarRunning(next);
    RobotAPI.lidar(next ? "start" : "stop").catch(() => {});
  };

  /* ===== UI ===== */

  return (
    <div className="space-y-6">
      {/* FPV video */}
      <div className="relative overflow-hidden rounded-2xl border border-white/10 bg-black shadow-[0_10px_30px_rgba(0,0,0,0.35)]">
        <div className="absolute left-3 top-2 text-green-300 text-xl font-bold drop-shadow">
          FPS:{fps}
        </div>
        <img
          src={streamUrl || "/placeholder.svg?height=360&width=640"}
          alt="FPV"
          className="w-full aspect-[16/7] object-cover"
        />
      </div>

      {/* 2 cột: trái = body, phải = move + poses */}
      <div className="grid grid-cols-1 xl:grid-cols-2 gap-6 items-start">
        {/* LEFT — Body Adjustment */}
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

        {/* RIGHT — Move + Posture/Behavior */}
        <div className="flex flex-col gap-6">
          <Panel title="Basic Postures">
            <div className="grid grid-cols-[repeat(auto-fit,minmax(120px,1fr))] gap-3">
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
            <div className="grid grid-cols-[repeat(auto-fit,minmax(120px,1fr))] gap-3">
              {axisMotionBtns.map((b) => (
                <Btn
                  key={b}
                  label={b.replaceAll("_", " ")}
                  onClick={() => RobotAPI.behavior(b)}
                />
              ))}
            </div>
          </Panel>

          <Panel title="Behavior Control">
            <div className="grid grid-cols-[repeat(auto-fit,minmax(140px,1fr))] gap-3">
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
    </div>
  );
}

/* ===== UI helpers ===== */

function Panel({
  title,
  children,
}: {
  title: string;
  children: React.ReactNode;
}) {
  return (
    <div className="rounded-2xl bg-white/5 border border-white/10 p-5 shadow-[inset_0_1px_0_rgba(255,255,255,0.05)]">
      <div className="text-base font-semibold mb-4 opacity-90">{title}</div>
      {children}
    </div>
  );
}

function Btn({ label, onClick }: { label: string; onClick?: () => void }) {
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
