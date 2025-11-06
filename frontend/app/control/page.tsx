"use client";
import { useState } from "react";
import { HalfCircleJoystick } from "@/components/HalfCircleJoystick";
import header_control from "@/components/header_control";
export default function RemoteControlMode() {
    const [speed, setSpeed] = useState<"slow" | "normal" | "high">("normal");
    const [fps] = useState(30);
    const [sliders, setSliders] = useState({
        tx: 0,
        ty: 0,
        tz: 0,
        rx: 0,
        ry: 0,
        rz: 0,
    });
    const [fpv, setFpv] = useState(false);

    const postureBtns = [
        "Lie_Down",
        "Stand_Up",
        "Sit_Down",
        "Squat",
        "Crawl",
    ];
    const axisMotionBtns = [
        "Turn_Roll",
        "Turn_Pitch",
        "Turn_Yaw",
        "3_Axis",
        "Turn_Around",
    ];
    const behavior1 = ["Wave_Hand", "Handshake", "Pray", "Stretch", "Swing"];
    const behavior2 = ["Wave_Body", "Handshake", "Pee", "Play_Ball", "Mark_Time"];



    const [isRunning, setIsRunning] = useState(false);

    const handleToggleLidar = () => {
        setIsRunning((prev) => !prev);
    };

    return (
        <section className="min-h-screen w-full bg-[#0c0520] text-white p-6">
            {/* Header */}
            {header_control()}

            {/* Main content row */}
            <div className="mt-6 grid grid-cols-1 lg:grid-cols-3 gap-6">
                {/* FPV video */}
                <div className="lg:col-span-2">
                    <div className="relative overflow-hidden rounded-2xl border border-white/10 bg-black">
                        <div className="absolute left-3 top-2 text-green-300 text-xl font-bold drop-shadow">FPS:{fps}</div>
                        <img
                            src="/placeholder.svg?height=360&width=640"
                            alt="FPV"
                            className="w-full aspect-video object-cover opacity-80"
                        />
                    </div>
                </div>

                {/* Right control panel */}
                <div className="space-y-6">
                    {/* Speed */}
                    <Panel title="Speed">
                        <div className="flex gap-3">
                            {(["slow", "normal", "high"] as const).map((s) => (
                                <Chip
                                    key={s}
                                    label={s.charAt(0).toUpperCase() + s.slice(1)}
                                    active={speed === s}
                                    onClick={() => setSpeed(s)}
                                />
                            ))}
                        </div>
                    </Panel>

                    <Panel title="Move">

                        <div className="grid grid-cols-1 sm:grid-cols-[auto_1fr] gap-4 items-start">
                            {/* Joystick cột trái */}
                            <div className="justify-self-center sm:justify-self-start">
                                <HalfCircleJoystick
                                    width={260}
                                    height={160}
                                    rest="center"   
                                    onChange={({ angleDeg, power }) => { /* ... */ }}
                                    onRelease={() => { /* ... */ }}
                                />

                            </div>

                            <div className="grid grid-cols-2 gap-2">
                                <Btn label="Turn left" onClick={() => {

                                }} />
                                <Btn label="Turn right" onClick={() => {

                                }} />
                                <Btn variant="danger" label="Stop" onClick={() => {
                      
                                }} />
                                <Btn
                                    variant={isRunning ? "danger" : "success"}
                                    label={isRunning ? "Stop Lidar" : "Start Lidar"}
                                    onClick={handleToggleLidar}
                                />
                            </div>
                        </div>
                    </Panel>


                    {/* Body Adjustment */}
                    <Panel title="Body Adjustment">
                        <SliderRow
                            label="Translation_X"
                            value={sliders.tx}
                            onChange={(v) => setSliders({ ...sliders, tx: v })}
                        />
                        <SliderRow
                            label="Translation_Y"
                            value={sliders.ty}
                            onChange={(v) => setSliders({ ...sliders, ty: v })}
                        />
                        <SliderRow
                            label="Translation_Z"
                            value={sliders.tz}
                            onChange={(v) => setSliders({ ...sliders, tz: v })}
                        />
                        <SliderRow
                            label="Rotation_X"
                            value={sliders.rx}
                            onChange={(v) => setSliders({ ...sliders, rx: v })}
                        />
                        <SliderRow
                            label="Rotation_Y"
                            value={sliders.ry}
                            onChange={(v) => setSliders({ ...sliders, ry: v })}
                        />
                        <SliderRow
                            label="Rotation_Z"
                            value={sliders.rz}
                            onChange={(v) => setSliders({ ...sliders, rz: v })}
                        />
                    </Panel>
                </div>
            </div>

            {/* Bottom control grids */}
            <div className="mt-8 grid grid-cols-1 lg:grid-cols-2 gap-6">
                {/* Basic Postures */}
                <Panel title="Basic Postures">
                    <div className="grid grid-cols-2 sm:grid-cols-5 gap-3">
                        {postureBtns.map((b) => (
                            <Btn key={b} label={b.replaceAll("_", " ")} />
                        ))}
                    </div>
                </Panel>

                {/* Axis Motion */}
                <Panel title="Axis Motion">
                    <div className="grid grid-cols-2 sm:grid-cols-5 gap-3">
                        {axisMotionBtns.map((b) => (
                            <Btn key={b} label={b.replaceAll("_", " ")} />
                        ))}
                    </div>
                </Panel>
            </div>

            {/* Behavior control */}
            <div className="mt-6">
                <Panel title="Behavior Control">
                    <div className="grid grid-cols-2 sm:grid-cols-10 gap-3">
                        {[...behavior1, ...behavior2].map((b) => (
                            <Btn key={b} label={b.replaceAll("_", " ")} />
                        ))}
                    </div>
                </Panel>
            </div>

        </section>
    );
}


function Panel({ title, children }: { title: string; children: React.ReactNode }) {
    return (
        <div className="rounded-2xl bg-white/5 border border-white/10 p-4">
            <div className="text-sm mb-3 opacity-80">{title}</div>
            {children}
        </div>
    );
}

function Chip({ label, active, onClick }: { label: string; active?: boolean; onClick?: () => void }) {
    return (
        <button
            onClick={onClick}
            className={`px-4 py-1 rounded-xl text-sm border transition 
      ${active ? "bg-indigo-500/30 border-indigo-400/40" : "bg-white/5 border-white/10 hover:bg-white/10"}`}
        >
            {label}
        </button>
    );
}

function Btn({ label, variant = "default", onClick }: { label: string; variant?: "default" | "danger" | "success"; onClick?: () => void }) {
    const base = "px-4 py-2 text-sm rounded-xl border transition";
    const styles =
        variant === "danger"
            ? "bg-rose-500/20 border-rose-400/30 hover:bg-rose-500/30"
            : variant === "success"
                ? "bg-emerald-500/20 border-emerald-400/30 hover:bg-emerald-500/30"
                : "bg-white/5 border-white/10 hover:bg-white/10";
    return (
        <button onClick={onClick} className={`${base} ${styles}`}>
            {label}
        </button>
    );
}

function SliderRow({ label, value, onChange }: { label: string; value: number; onChange: (v: number) => void }) {
    return (
        <div className="mb-3">
            <div className="text-xs mb-1 opacity-75">{label}</div>
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

function Joystick({
    onChange,
    onRelease,
}: {
    onChange: (v: { angleDeg: number; power: number }) => void;
    onRelease: () => void;
}) {
    return (
        <div className="w-[220px]">
            <HalfCircleJoystick onChange={onChange} onRelease={onRelease} />
        </div>
    );
}


function JoyBtn({ label }: { label: string }) {
    return (
        <button className="w-8 h-8 rounded-full grid place-items-center text-sm bg-white/10 border border-white/10 hover:bg-white/20" aria-label={label}>
            {label}
        </button>
    );
}
