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

            </div>
        </>
    );
}







