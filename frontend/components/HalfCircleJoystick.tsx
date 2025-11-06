"use client";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";

// ==========================
// Tailwind Half-Circle Joystick
// - SVG-based, sleek UI
// - angleDeg range: [-90 .. +90]
// - power range: [0 .. 1]
// ==========================

type JoystickChange = { angleDeg: number; power: number };

type HalfCircleJoystickProps = {
  width?: number; // px
  height?: number; // px
  knobRadius?: number; // px
  onChange?: (data: JoystickChange) => void;
  onRelease?: () => void;
  disabled?: boolean;
  className?: string;
  /** starting knob position: "center" (default), "top", or custom {x,y} */
  rest?: "center" | "top" | { x: number; y: number };
};

function clamp(v: number, min: number, max: number) {
  return Math.max(min, Math.min(max, v));
}

function polarToDeg(x: number, y: number) {
  return (Math.atan2(y, x) * 180) / Math.PI;
}

function HalfCircleJoystick({
  width = 260,
  height = 160,
  knobRadius = 14,
  onChange,
  onRelease,
  disabled = false,
  className = "",
  rest = "center",
}: HalfCircleJoystickProps) {
  // geometry basics
  const pad = 12;
  const cx = width / 2;
  const cy = height - pad; // sit on the bottom
  const R = Math.min(cx - pad, cy - pad);

  const svgRef = useRef<SVGSVGElement | null>(null);
  const dragging = useRef(false);
  const [knob, setKnob] = useState<{ x: number; y: number } | null>(null);
  const [isPressed, setIsPressed] = useState(false);

  // default knob rest position
  const restPos = useMemo(() => {
    if (typeof rest === "object") return rest;
    if (rest === "top") return { x: cx, y: cy - R }; // on the arc top
    // default: true center of the semicircle area
    return { x: cx, y: cy - R * 0.5 };
  }, [cx, cy, R, rest]);

  // computed arc path for the half circle (180°)
  const arcPath = useMemo(() => {
    const x1 = cx - R,
      y1 = cy;
    const x2 = cx + R,
      y2 = cy;
    // large-arc-flag=1, sweep-flag=0 (counterclockwise) gives upper semicircle
    return `M ${x1} ${y1} A ${R} ${R} 0 1 0 ${x2} ${y2}`;
  }, [cx, cy, R]);

  // map pointer to constrained point on/inside half circle
  const getLocalPoint = useCallback(
    (clientX: number, clientY: number) => {
      const bbox = svgRef.current!.getBoundingClientRect();
      const x = clientX - bbox.left;
      const y = clientY - bbox.top;
      return { x, y };
    },
    []
  );

  const projectToSemicircle = useCallback(
    (pt: { x: number; y: number }) => {
      let dx = pt.x - cx;
      let dy = pt.y - cy;
      // forbid going below baseline (lower half). Keep y <= cy
      dy = Math.min(dy, 0);
      const dist = Math.hypot(dx, dy);
      if (dist <= R && dy <= 0) return { x: pt.x, y: pt.y };
      // project to circle boundary when outside
      const k = R / (dist || 1);
      return { x: cx + dx * k, y: cy + dy * k };
    },
    [cx, cy, R]
  );

  const computeOutput = useCallback(
    (p: { x: number; y: number }) => {
      const vx = p.x - cx;
      const vy = p.y - cy; // vy is negative in the upper half
      const ang = polarToDeg(vx, vy); // -180..+180 with 0° on +x axis
      // For upper half, ang in [-180..0]; map to [-90..+90] with 0 at up
      // We'll compute joystick angle where left = -90, right = +90, center up = 0.
      // Convert by: angleDeg = - (ang + 90)
      // Explanation: at leftmost (-180), -> +90; at up (0, -1) ang=-90 -> 0; at right (0) -> -90 => flip sign
      const angleDeg = clamp(-(ang + 90), -90, 90);
      const power = clamp(Math.hypot(vx, vy) / R, 0, 1);
      return { angleDeg, power } as JoystickChange;
    },
    [cx, cy, R]
  );

  const setFromPointer = useCallback(
    (clientX: number, clientY: number) => {
      if (!svgRef.current) return;
      const local = getLocalPoint(clientX, clientY);
      const p = projectToSemicircle(local);
      setKnob({ x: p.x, y: p.y });
      onChange?.(computeOutput(p));
    },
    [computeOutput, getLocalPoint, onChange, projectToSemicircle]
  );

  const handlePointerDown = (e: React.PointerEvent) => {
    if (disabled) return;
    (e.target as Element).setPointerCapture?.(e.pointerId);
    dragging.current = true;
    setIsPressed(true);
    setFromPointer(e.clientX, e.clientY);
  };

  const handlePointerMove = (e: PointerEvent) => {
    if (!dragging.current || disabled) return;
    setFromPointer(e.clientX, e.clientY);
  };

  const endDrag = useCallback(() => {
    if (!dragging.current) return;
    dragging.current = false;
    setIsPressed(false);
    setKnob(null); // spring back to rest (animated via CSS)
    onRelease?.();
  }, [onRelease]);

  useEffect(() => {
    window.addEventListener("pointermove", handlePointerMove);
    window.addEventListener("pointerup", endDrag);
    window.addEventListener("pointercancel", endDrag);
    return () => {
      window.removeEventListener("pointermove", handlePointerMove);
      window.removeEventListener("pointerup", endDrag);
      window.removeEventListener("pointercancel", endDrag);
    };
  }, [endDrag]);

  const knobPos = knob ?? restPos;

  return (
    <div
      className={`select-none ${disabled ? "opacity-50" : ""} ${className}`}
      aria-disabled={disabled}
    >
      <svg
        ref={svgRef}
        width={width}
        height={height}
        viewBox={`0 0 ${width} ${height}`}
        className="rounded-2xl shadow-sm bg-gradient-to-b from-slate-900 via-slate-900 to-slate-800 ring-1 ring-slate-700/60"
        onPointerDown={handlePointerDown}
        role="slider"
        aria-valuemin={-90}
        aria-valuemax={90}
        aria-label="Half-circle joystick"
      >
        {/* defs for nice glow */}
        <defs>
          <radialGradient id="padGlow" cx="50%" cy="60%" r="60%">
            <stop offset="0%" stopColor="#2dd4bf" stopOpacity="0.25" />
            <stop offset="100%" stopColor="#22d3ee" stopOpacity="0" />
          </radialGradient>
          <linearGradient id="track" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#93c5fd" />
            <stop offset="100%" stopColor="#22d3ee" />
          </linearGradient>
        </defs>

        {/* glow/fill area */}
        <path d={arcPath} fill="url(#padGlow)" stroke="none" />

        {/* track (arc outline) */}
        <path
          d={arcPath}
          className="[stroke:url(#track)]"
          strokeWidth={6}
          fill="none"
          strokeLinecap="round"
        />

        {/* tick marks */}
        {[-90, -60, -30, 0, 30, 60, 90].map((deg, i) => {
          // snap to whole pixels for crisper, perfectly centered ticks
          const rad = ((deg + 90) * Math.PI) / 180; // map -90..90 to 0..180
          const fx = (n: number) => Math.round(n) + 0.5;
          const x = fx(cx + R * Math.cos(rad));
          const y = fx(cy - R * Math.sin(rad));
          const inner = 10;
          const x2 = fx(cx + (R - inner) * Math.cos(rad));
          const y2 = fx(cy - (R - inner) * Math.sin(rad));
          return (
            <line
              key={i}
              x1={x}
              y1={y}
              x2={x2}
              y2={y2}
              className="stroke-slate-400/60"
              strokeWidth={deg === 0 ? 2.5 : 1.5}
              strokeLinecap="round"
            />
          );
        })}

        {/* dynamic guide from center to knob */}
        <line
          x1={cx}
          y1={cy}
          x2={knobPos.x}
          y2={knobPos.y}
          className={`${
            isPressed ? "opacity-80" : "opacity-40"
          } stroke-cyan-300`}
          strokeWidth={2}
          strokeLinecap="round"
        />

        {/* knob */}
        <g
          className={`transition-transform duration-150 ease-out ${
            isPressed ? "scale-100" : "scale-95"
          }`}
        >
          <circle
            cx={knobPos.x}
            cy={knobPos.y}
            r={knobRadius}
            className="fill-white/95"
            style={{ filter: "drop-shadow(0 6px 10px rgba(0,0,0,0.35))" }}
          />
          <circle
            cx={knobPos.x}
            cy={knobPos.y}
            r={knobRadius - 5}
            className="fill-cyan-400/90"
          />
          <circle cx={knobPos.x} cy={knobPos.y} r={3} className="fill-white" />
        </g>
      </svg>
    </div>
  );
}

// ==========================
// Demo wrapper (default export)
// ==========================
export default function JoystickDemo() {
  const [state, setState] = useState<JoystickChange>({ angleDeg: 0, power: 0 });
  return (
    <div className="min-h-[60vh] w-full flex items-center justify-center bg-slate-950 p-6">
      <div className="w-full max-w-xl grid md:grid-cols-[1fr_260px] gap-6 items-center">
        <div className="rounded-2xl border border-slate-800/80 bg-slate-900/60 p-5 shadow-xl">
          <div className="flex items-center justify-between">
            <h2 className="text-lg font-semibold text-slate-100">Half‑Circle Joystick</h2>
            <span className="text-xs text-slate-400">Tailwind + SVG</span>
          </div>
          <div className="mt-4 flex justify-center">
            <HalfCircleJoystick
              width={260}
              height={160}
              onChange={(d) => setState(d)}
              onRelease={() => setState((s) => ({ ...s, power: 0 }))}
            />
          </div>
        </div>

        <div className="rounded-2xl border border-slate-800/80 bg-slate-900/60 p-5 shadow-xl">
          <h3 className="text-sm font-medium text-slate-200">Output</h3>
          <div className="mt-3 grid grid-cols-2 gap-3">
            <div className="rounded-xl bg-slate-800 px-3 py-2">
              <div className="text-[10px] uppercase tracking-wide text-slate-400">Angle</div>
              <div className="text-2xl font-semibold text-slate-50 tabular-nums">
                {state.angleDeg.toFixed(0)}°
              </div>
            </div>
            <div className="rounded-xl bg-slate-800 px-3 py-2">
              <div className="text-[10px] uppercase tracking-wide text-slate-400">Power</div>
              <div className="text-2xl font-semibold text-slate-50 tabular-nums">
                {(state.power * 100).toFixed(0)}%
              </div>
            </div>
          </div>
          <p className="mt-3 text-xs text-slate-400">
            Drag the knob across the upper semicircle. Release to spring back.
          </p>
        </div>
      </div>
    </div>
  );
}

export { HalfCircleJoystick };
