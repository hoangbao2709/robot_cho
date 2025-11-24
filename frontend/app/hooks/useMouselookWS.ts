// app/hooks/useMouselookWS.ts
"use client";
import { useEffect, useRef, useState, useCallback } from "react";

export function useMouselookWS(robotId: string) {
  const [enabled, setEnabled] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);
  const lastSentRef = useRef<number>(0);
  const HZ = 60; // limit gửi 60Hz

  const connect = useCallback(() => {
    const url = (process.env.NEXT_PUBLIC_WS_BASE || "ws://127.0.0.1:8000") + `/ws/robots/${robotId}/control/`;
    const ws = new WebSocket(url);
    wsRef.current = ws;
    ws.onopen = () => console.log("[WS] control connected");
    ws.onclose = () => console.log("[WS] control closed");
    ws.onerror = (e) => console.log("[WS] control error", e);
    ws.onmessage = (ev) => {
      // console.log("WS RX:", ev.data)
    };
  }, [robotId]);

  useEffect(() => {
    connect();
    return () => wsRef.current?.close();
  }, [connect]);

  // bật/tắt mouselook
  const setEnable = useCallback((flag: boolean) => {
    setEnabled(flag);
    wsRef.current?.send(JSON.stringify({ type: "ml_enable", enable: flag }));
  }, []);

  // gửi movementX/Y (đã pointer-lock)
  const sendMove = useCallback((dx: number, dy: number) => {
    const now = performance.now();
    if (now - lastSentRef.current < 1000 / HZ) return;
    lastSentRef.current = now;
    wsRef.current?.send(JSON.stringify({ type: "ml_move", dx, dy }));
  }, []);

  const stop = useCallback(() => {
    wsRef.current?.send(JSON.stringify({ type: "stop" }));
  }, []);

  return { enabled, setEnable, sendMove, stop };
}
