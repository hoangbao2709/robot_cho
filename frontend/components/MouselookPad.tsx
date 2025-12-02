"use client";

import { useEffect, useRef, useState } from "react";
import { RobotAPI } from "@/app/lib/robotApi";

type Props = {
  robotId: string;
  enabled: boolean;
};

const LIN_V = 0.35;
const WS_ORIGIN =
  process.env.NEXT_PUBLIC_BACKEND_WS || "ws://127.0.0.1:8000";

export default function MouselookPad({ robotId, enabled }: Props) {
  const overlayRef = useRef<HTMLDivElement | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const wsReadyRef = useRef(false);

  const [ctrlHeld, setCtrlHeld] = useState(false);

  // fwd = W/S, turn = quay trái/phải bằng A/D
  const velRef = useRef<{ fwd: number; turn: number }>({ fwd: 0, turn: 0 });

  // ===== WebSocket =====
  useEffect(() => {
    if (!enabled) {
      wsRef.current?.close();
      wsRef.current = null;
      wsReadyRef.current = false;
      stopMove();
      return;
    }

    const wsUrl = `${WS_ORIGIN}/ws/robots/${robotId}/control/`;
    console.log("[MouseLook] connect WS =>", wsUrl);

    const ws = new WebSocket(wsUrl);
    wsRef.current = ws;
    wsReadyRef.current = false;

    ws.onopen = () => {
      console.log("[MouseLook] WS OPEN");
      wsReadyRef.current = true;
      ws.send(JSON.stringify({ type: "ml_enable", enable: true }));
    };

    ws.onclose = (ev) => {
      console.log("[MouseLook] WS CLOSE", ev.code, ev.reason);
      wsReadyRef.current = false;
      wsRef.current = null;
      stopMove();
    };

    ws.onerror = (err) => {
      console.error("[MouseLook] WS ERROR", err);
      wsReadyRef.current = false;
    };

    return () => {
      ws.close();
    };
  }, [enabled, robotId]);

  // ===== Mouse move => gửi dx,dy cho server =====
  const handleMouseMove = (e: React.MouseEvent<HTMLDivElement>) => {
    if (!enabled) return;
    if (ctrlHeld) return; // đang giữ Ctrl để thao tác UI

    const ws = wsRef.current;
    if (!ws || !wsReadyRef.current || ws.readyState !== WebSocket.OPEN) return;

    const { movementX, movementY } = e.nativeEvent;
    if (movementX === 0 && movementY === 0) return;

    // DEBUG: xem log trên console trước
    // console.log("ml_move", movementX, movementY);

    ws.send(
      JSON.stringify({
        type: "ml_move",
        dx: movementX,
        dy: movementY,
      })
    );
  };

  // ===== WASD + Ctrl =====
  useEffect(() => {
    if (!enabled) return;

    const recalcMove = () => {
      const { fwd, turn } = velRef.current;
      const vx = fwd * LIN_V;
      const rz = turn * 1.0;

      RobotAPI.move({ vx, vy: 0, vz: 0, rx: 0, ry: 0, rz }).catch(() => {});
    };

    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();

      // Đang gõ trong input/textarea thì bỏ qua
      const target = e.target as HTMLElement | null;
      if (
        target &&
        (target.tagName === "INPUT" ||
          target.tagName === "TEXTAREA" ||
          target.getAttribute("contenteditable") === "true")
      ) {
        return;
      }

      if (key === "control") {
        if (!ctrlHeld) setCtrlHeld(true);
        return;
      }

      if (!enabled || ctrlHeld) return;

      let changed = false;
      switch (key) {
        case "w":
          if (velRef.current.fwd !== 1) {
            velRef.current.fwd = 1;
            changed = true;
          }
          break;
        case "s":
          if (velRef.current.fwd !== -1) {
            velRef.current.fwd = -1;
            changed = true;
          }
          break;
        case "a":
          if (velRef.current.turn !== 1) {
            velRef.current.turn = 1;
            changed = true;
          }
          break;
        case "d":
          if (velRef.current.turn !== -1) {
            velRef.current.turn = -1;
            changed = true;
          }
          break;
        default:
          return;
      }

      if (changed) {
        e.preventDefault();
        recalcMove();
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();

      if (key === "control") {
        if (ctrlHeld) setCtrlHeld(false);
        return;
      }

      if (!enabled || ctrlHeld) return;

      let changed = false;
      switch (key) {
        case "w":
        case "s":
          if (velRef.current.fwd !== 0) {
            velRef.current.fwd = 0;
            changed = true;
          }
          break;
        case "a":
        case "d":
          if (velRef.current.turn !== 0) {
            velRef.current.turn = 0;
            changed = true;
          }
          break;
        default:
          return;
      }

      if (changed) {
        e.preventDefault();
        recalcMove();
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      stopMove();
    };
  }, [enabled, ctrlHeld]);

  const stopMove = () => {
    velRef.current = { fwd: 0, turn: 0 };
    RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: 0 }).catch(() => {});
  };

  if (!enabled) return null;

  return (
    <div
      ref={overlayRef}
      onMouseMove={handleMouseMove}
      className={`fixed inset-0 z-40 ${ctrlHeld ? "" : "cursor-none"}`}
      style={{
        background: "transparent",
        // Giữ Ctrl => không chặn click UI
        pointerEvents: ctrlHeld ? "none" : "auto",
      }}
    />
  );
}
