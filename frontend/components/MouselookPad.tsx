"use client";

import { useEffect, useRef, useState } from "react";
import { RobotAPI } from "@/app/lib/robotApi";

type Props = {
  robotId: string;
  enabled: boolean;
};

const LIN_V = 0.35;
const ANG_W = 1.0;

const WS_ORIGIN =
  process.env.NEXT_PUBLIC_BACKEND_WS || "ws://127.0.0.1:8000";

export default function MouselookPad({ robotId, enabled }: Props) {
  const overlayRef = useRef<HTMLDivElement | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const wsReadyRef = useRef(false);
  const pointerLockedRef = useRef(false);

  const [locked, setLocked] = useState(false);
  const [ctrlHeld, setCtrlHeld] = useState(false);

  const velRef = useRef<{ fwd: number; turn: number }>({ fwd: 0, turn: 0 });

  // ===== WebSocket =====
  useEffect(() => {
    if (!enabled) {
      wsRef.current?.close();
      wsRef.current = null;
      wsReadyRef.current = false;
      pointerLockedRef.current = false;
      setLocked(false);
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
      pointerLockedRef.current = false;
      setLocked(false);
    };

    ws.onerror = (err) => {
      console.error("[MouseLook] WS ERROR", err);
      wsReadyRef.current = false;
    };

    return () => {
      ws.close();
    };
  }, [enabled, robotId]);

  // ===== Mouse move => dx,dy =====
  useEffect(() => {
    if (!enabled) return;

    const handleMouseMove = (e: MouseEvent) => {
      if (!pointerLockedRef.current) return;
      const ws = wsRef.current;
      if (!ws || !wsReadyRef.current || ws.readyState !== WebSocket.OPEN) return;

      const { movementX, movementY } = e;
      if (movementX === 0 && movementY === 0) return;

      ws.send(
        JSON.stringify({
          type: "ml_move",
          dx: movementX,
          dy: movementY,
        })
      );
    };

    const handleLockChange = () => {
      const el = overlayRef.current;
      const isLocked = document.pointerLockElement === el;
      pointerLockedRef.current = isLocked;
      setLocked(isLocked);
      console.log("[MouseLook] pointer locked:", isLocked);
      if (!isLocked) {
        velRef.current.turn = 0;
        sendMoveOnce();
      }
    };

    document.addEventListener("mousemove", handleMouseMove);
    document.addEventListener("pointerlockchange", handleLockChange);

    return () => {
      document.removeEventListener("mousemove", handleMouseMove);
      document.removeEventListener("pointerlockchange", handleLockChange);
    };
  }, [enabled]);

  // ===== WASD + Ctrl =====
  useEffect(() => {
    if (!enabled) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();

      // Giữ Ctrl => thoát lock + cho click UI
      if (key === "control") {
        if (!ctrlHeld) {
          setCtrlHeld(true);
          if (pointerLockedRef.current) {
            document.exitPointerLock();
          }
        }
        return;
      }

      if (!pointerLockedRef.current) return;

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
        sendMoveOnce();
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();

      // Thả Ctrl => bật lại lock + ẩn chuột
      if (key === "control") {
        if (ctrlHeld) {
          setCtrlHeld(false);
          const el = overlayRef.current;
          if (el && enabled) {
            (el as any).requestPointerLock?.();
          }
        }
        return;
      }

      if (!pointerLockedRef.current) return;

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
        sendMoveOnce();
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

  const sendMoveOnce = () => {
    const { fwd, turn } = velRef.current;
    const vx = fwd * LIN_V;
    const rz = turn * ANG_W;
    RobotAPI.move({ vx, vy: 0, vz: 0, rx: 0, ry: 0, rz }).catch(() => {});
  };

  const stopMove = () => {
    velRef.current = { fwd: 0, turn: 0 };
    RobotAPI.move({ vx: 0, vy: 0, vz: 0, rx: 0, ry: 0, rz: 0 }).catch(() => {});
  };

  const requestLock = () => {
    if (!enabled || ctrlHeld) return;
    const el = overlayRef.current;
    if (!el) return;
    if (document.pointerLockElement === el) return;
    (el as any).requestPointerLock?.();
  };

  if (!enabled) return null;

  return (
    <div
      ref={overlayRef}
      onClick={requestLock}
      className={`fixed inset-0 z-40 ${ctrlHeld ? "" : "cursor-none"}`}
      style={{
        background: "transparent",
        pointerEvents: ctrlHeld ? "none" : "auto", // giữ Ctrl => không chặn click
      }}
    />
  );
}
