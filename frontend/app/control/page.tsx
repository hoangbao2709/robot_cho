"use client";
import React, { useCallback, useEffect, useState } from "react";
import RemoteView from "@/components/control/RemoteView";
import FPVView from "@/components/control/FPVView";
import HeaderControl from "@/components/header_control";

// demo; thay bằng API thật của bạn
async function robotStop() {
  try {
    await fetch("/api/robots/robot-a/command/move/stop", { method: "POST" });
  } catch {}
}
async function getFpv() {
  return { stream_url: "/placeholder.svg?height=360&width=640", fps: 30 };
}

export default function ManualControlPage() {
  const [mode, setMode] = useState<"remote" | "fpv">("remote");
  const [fpv, setFpv] = useState<{ stream_url?: string; fps?: number }>({});

  useEffect(() => {
    if (mode === "fpv") {
      robotStop();
      getFpv().then(setFpv).catch(() => {});
    }
  }, [mode]);

  const toggleMode = useCallback(
    () => setMode((m) => (m === "remote" ? "fpv" : "remote")),
    []
  );

  return (
    <section className="min-h-screen w-full bg-[#0c0520] text-white p-6">
      <div className="mt-6">
        {mode === "remote" ? (
          <RemoteView
            onEmergencyStop={robotStop}
            mode={mode}
            toggleMode={toggleMode}
          />
        ) : (
          <div className="space-y-4">
            <HeaderControl mode={mode} onToggle={toggleMode} connected={true} />
            <FPVView fps={fpv.fps ?? 30} />
          </div>
        )}
      </div>
    </section>
  );
}
