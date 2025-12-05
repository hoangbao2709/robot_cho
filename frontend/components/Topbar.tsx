"use client";

import { useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import Image from "next/image";

export default function Topbar() {
  const router = useRouter();
  const [username, setUsername] = useState<string | null>(null);

  useEffect(() => {
    const token = localStorage.getItem("access_token");
    const storedUser = localStorage.getItem("username");

    if (token && storedUser) {
      setUsername(storedUser);
      return;
    }

    // Nếu chỉ có token, lấy username từ payload JWT
    if (token) {
      try {
        const payload = JSON.parse(atob(token.split(".")[1]));
        const name = payload.username || "User";
        setUsername(name);
        localStorage.setItem("username", name);
      } catch {
        setUsername(null);
      }
    } else {
      setUsername(null);
    }
  }, []);

  function logout() {
    localStorage.removeItem("access_token");
    localStorage.removeItem("username");

    // animation nhỏ trước khi chuyển trang
    setTimeout(() => {
      router.push("/login");
    }, 200);
  }

  function goLogin() {
    router.push("/login");
  }

  return (
    <header className="border-b border-white/10 bg-[#1A0F28]/70 backdrop-blur p-4 flex justify-between items-center">
      <input
        placeholder="Search..."
        className="rounded-full bg-white/10 px-4 py-2 text-sm placeholder:text-white/60 focus:outline-none flex-1 max-w-md"
      />

      <div className="flex items-center gap-4 text-sm text-white/80">

        {!username && (
          <button
            onClick={goLogin}
            className="
              px-4 py-2 rounded-lg text-sm
              bg-violet-600/40 hover:bg-violet-600/60
              border border-violet-400/40
              text-white transition-all duration-200
              hover:scale-105 active:scale-95
              cursor-pointer
            "
          >
            Login
          </button>
        )}
        {username && (
          <>
            <span className="font-medium">{username}</span>


            <div className="h-8 w-8 rounded-full bg-white/20 border border-white/10">
              <Image
                src="/logo_hongtrang.png"
                alt="RobotControl Logo"
                width={60}
                height={60}
                className="rounded-full mb-3"
              />
            </div>

            <button
              onClick={logout}
              className="
                px-3 py-1 rounded-lg text-xs
                bg-white/10 hover:bg-red-500/40
                border border-white/10
                text-white transition-all duration-200
                hover:scale-105 active:scale-95
                shadow-sm hover:shadow-red-500/20
                cursor-pointer
              "
            >
              Logout
            </button>
          </>
        )}

      </div>
    </header>
  );
}
