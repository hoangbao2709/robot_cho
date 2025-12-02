"use client";

import React, { useEffect, useRef, useState } from "react";
import { RobotAPI } from "@/app/lib/robotApi";

type Props = {
  robotId: string;
  enabled: boolean;
};

const LIN_V = 0.35;      // tốc độ tịnh tiến tối đa
const YAW_SENS = 0.003;  // độ nhạy xoay chuột
const MAX_YAW = 1.0;     // giới hạn tốc độ quay

export default function MouselookPad({ enabled }: Props) {
  const overlayRef = useRef<HTMLDivElement | null>(null);
  const [locked, setLocked] = useState(false);
  const pointerLockedRef = useRef(false);

  // trạng thái phím W/S/A/D
  const keysRef = useRef({
    w: false,
    s: false,
    a: false,
    d: false,
  });

  // ===== helper: vx, vy từ W/S/A/D =====
  const getVelFromKeys = () => {
    const k = keysRef.current;

    let fwd = 0;
    if (k.w) fwd += 1;
    if (k.s) fwd -= 1;

    let strafe = 0;
    if (k.d) strafe += 1;
    if (k.a) strafe -= 1;

    if (fwd === 0 && strafe === 0) {
      return { vx: 0, vy: 0, active: false };
    }

    const len = Math.sqrt(fwd * fwd + strafe * strafe) || 1;
    fwd /= len;
    strafe /= len;

    return {
      vx: fwd * LIN_V,
      vy: strafe * LIN_V,
      active: true,
    };
  };

  const sendMove = (rz: number) => {
    const { vx, vy, active } = getVelFromKeys();
    if (!active && rz === 0) {
      RobotAPI.move({
        vx: 0,
        vy: 0,
        vz: 0,
        rx: 0,
        ry: 0,
        rz: 0,
      }).catch(() => {});
      return;
    }

    RobotAPI.move({
      vx,
      vy,
      vz: 0,
      rx: 0,
      ry: 0,
      rz,
    }).catch(() => {});
  };

  const stopMove = () => {
    keysRef.current = { w: false, s: false, a: false, d: false };
    RobotAPI.move({
      vx: 0,
      vy: 0,
      vz: 0,
      rx: 0,
      ry: 0,
      rz: 0,
    }).catch(() => {});
  };

  // ===== Pointer lock: mousemove trên document =====
  useEffect(() => {
    if (!enabled) {
      if (document.pointerLockElement === overlayRef.current) {
        document.exitPointerLock();
      }
      pointerLockedRef.current = false;
      setLocked(false);
      stopMove();
      return;
    }

    const handleMouseMove = (e: MouseEvent) => {
      if (!pointerLockedRef.current) return;

      const movementX = e.movementX || 0;
      if (!movementX) return;

      let rz = -movementX * YAW_SENS; // đảo dấu nếu ngược cảm giác
      if (rz > MAX_YAW) rz = MAX_YAW;
      if (rz < -MAX_YAW) rz = -MAX_YAW;

      sendMove(rz);
    };

    const handleLockChange = () => {
      const el = overlayRef.current;
      const isLocked = document.pointerLockElement === el;
      pointerLockedRef.current = isLocked;
      setLocked(isLocked);
      if (!isLocked) {
        stopMove();
      }
    };

    document.addEventListener("mousemove", handleMouseMove);
    document.addEventListener("pointerlockchange", handleLockChange);

    return () => {
      document.removeEventListener("mousemove", handleMouseMove);
      document.removeEventListener("pointerlockchange", handleLockChange);
      if (document.pointerLockElement === overlayRef.current) {
        document.exitPointerLock();
      }
      pointerLockedRef.current = false;
      setLocked(false);
      stopMove();
    };
  }, [enabled]);

  // ===== WASD =====
  useEffect(() => {
    if (!enabled) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();

      const target = e.target as HTMLElement | null;
      if (
        target &&
        (target.tagName === "INPUT" ||
          target.tagName === "TEXTAREA" ||
          target.getAttribute("contenteditable") === "true")
      ) {
        return;
      }

      if (!pointerLockedRef.current) return;

      let changed = false;
      switch (key) {
        case "w":
          if (!keysRef.current.w) {
            keysRef.current.w = true;
            changed = true;
          }
          break;
        case "s":
          if (!keysRef.current.s) {
            keysRef.current.s = true;
            changed = true;
          }
          break;
        case "a":
          if (!keysRef.current.a) {
            keysRef.current.a = true;
            changed = true;
          }
          break;
        case "d":
          if (!keysRef.current.d) {
            keysRef.current.d = true;
            changed = true;
          }
          break;
        default:
          return;
      }

      if (changed) {
        e.preventDefault();
        sendMove(0); // chỉ update vx, vy
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();

      if (!pointerLockedRef.current) return;

      let changed = false;
      switch (key) {
        case "w":
          if (keysRef.current.w) {
            keysRef.current.w = false;
            changed = true;
          }
          break;
        case "s":
          if (keysRef.current.s) {
            keysRef.current.s = false;
            changed = true;
          }
          break;
        case "a":
          if (keysRef.current.a) {
            keysRef.current.a = false;
            changed = true;
          }
          break;
        case "d":
          if (keysRef.current.d) {
            keysRef.current.d = false;
            changed = true;
          }
          break;
        default:
          return;
      }

      if (changed) {
        e.preventDefault();
        sendMove(0);
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      stopMove();
    };
  }, [enabled]);

  // ===== click để toggle lock trong khung FPV =====
  const toggleLock = () => {
    if (!enabled) return;
    const el = overlayRef.current;
    if (!el) return;

    if (pointerLockedRef.current) {
      document.exitPointerLock();
    } else {
      (el as any).requestPointerLock?.();
    }
  };

  if (!enabled) return null;

  return (
    <div
      ref={overlayRef}
      onClick={toggleLock}
      className={`absolute inset-0 ${
        locked ? "cursor-none" : "cursor-crosshair"
      }`}
      style={{
        background: "transparent",
      }}
    />
  );
}
