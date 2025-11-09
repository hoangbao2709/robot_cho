"use client";
import Link from "next/link";
import Image from "next/image";
import { usePathname } from "next/navigation";
import {
  Link2,
  Gamepad2,
  Bot,
  BarChart3,
  LogOut,
} from "lucide-react"; // icon đẹp, cài: npm i lucide-react

const menu = [
  { href: "/dashboard", label: "Connection", icon: Link2 },
  { href: "/control", label: "Manual Control", icon: Gamepad2 },
  { href: "/autonomous", label: "Autonomous Control", icon: Bot },
  { href: "/analytics", label: "Analytics", icon: BarChart3 },
];

export default function Sidebar() {
  const path = usePathname();

  return (
    <aside className="w-64 h-screen bg-[#160626] border-r border-white/10 flex flex-col justify-between">
      {/* --- Logo & Title --- */}
      <div className="flex flex-col items-center pt-8 pb-6">
        <Image
          src="/logo_hongtrang.png"
          alt="RobotControl Logo"
          width={60}
          height={60}
          className="rounded-full mb-3"
        />
        <h1 className="text-pink-400 font-bold text-xl tracking-wide">
          RobotControl
        </h1>
      </div>

      {/* --- Menu --- */}
      <nav className="flex-1 w-full mt-4 space-y-1 px-5">
        {menu.map((item) => {
          const Icon = item.icon;
          const active = path.startsWith(item.href);
          return (
            <Link
              key={item.href}
              href={item.href}
              className={`flex items-center gap-3 rounded-xl px-4 py-4 text-sm font-medium transition-all ${
                active
                  ? "bg-gradient-to-r from-pink-500 to-purple-600 text-white shadow-md"
                  : "text-white/70 hover:bg-white/5 hover:text-white"
              }`}
            >
              <Icon size={18} />
              {item.label}
            </Link>
          );
        })}
      </nav>

      {/* --- Logout --- */}
      <div className="p-5 border-t border-white/10">
        <button className="flex items-center gap-3 text-sm text-white/70 hover:text-white transition">
          <LogOut size={18} />
          Log Out
        </button>
      </div>
    </aside>
  );
}
