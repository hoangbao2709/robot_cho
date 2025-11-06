"use client";
import { useState } from "react";

export default function header_control() {
    const [fpv, setFpv] = useState(false);
    function FpvToggle({ on, onToggle }: { on: boolean; onToggle: () => void }) {
        return (
            <button
                onClick={onToggle}
                className={`w-11 h-7 rounded-full border border-violet-400/50 p-1 grid items-center transition ${on ? "bg-violet-500/30" : "bg-transparent"
                    }`}
                aria-label="Toggle FPV"
            >
                <div
                    className={`w-5 h-5 rounded-full border border-violet-300/60 bg-white/10 transition-transform ${on ? "translate-x-4" : ""
                        }`}
                />
            </button>
        );
    }
    function Metric({ label, value }: { label: string; value: string }) {
        return (
            <div className="rounded-xl bg-white/5 border border-white/10 p-3">
                <div className="text-[10px] uppercase opacity-60">{label}</div>
                <div className="text-sm mt-1">{value}</div>
            </div>
        );
    }

    return (
        <>
            <header className="flex items-center justify-between">
                <h1 className="text-2xl font-bold tracking-tight">
                    <span className="text-pink-400">REMOTE</span>{" "}
                    <span className="text-sky-300">CONTROL</span>{" "}
                    <span className="text-indigo-400">MODE</span>
                </h1>
                <div className="flex items-center gap-3">
                    <button className="px-3 py-1 rounded-xl bg-pink-500/20 hover:bg-pink-500/30 text-pink-300 text-sm">
                        Disconnect
                    </button>
                    <span className="text-sm opacity-70">Remote</span>
                    <FpvToggle on={fpv} onToggle={() => setFpv(!fpv)} />
                    <span className="text-sm">FPV</span>
                </div>

            </header>

            {/* Robot header row */}
            <div className="mt-4 grid grid-cols-1 lg:grid-cols-3 gap-6">
                {/* Robot A + metrics */}
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

                {/* Lidar map */}
                <div className="p-4 rounded-2xl bg-white/5 border border-white/10">
                    <div className="text-sm mb-2 opacity-80">Lidar map</div>
                    <div className="h-40 rounded-xl bg-gradient-to-b from-slate-900 to-slate-800 border border-white/10" />
                </div>
            </div>
        </>
    );
}







