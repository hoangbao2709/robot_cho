"use client";
import Link from "next/link";
import { usePathname } from "next/navigation";

const menu = [
  { href: "/connection", label: "Connection" },
  { href: "/teleop", label: "Manual Control" },
  { href: "/autonomous", label: "Autonomous Control" },
  { href: "/analytics", label: "Analytics" },
];

export default function Sidebar() {
  const path = usePathname();

  return (
    <aside className="w-64 bg-[#221133] border-r border-white/10 flex flex-col">
      <div className="p-4 text-pink-400 font-bold text-lg">🤖 RobotControl</div>
      <nav className="flex-1 space-y-1 px-3">
        {menu.map((item) => (
          <Link
            key={item.href}
            href={item.href}
            className={`block rounded-xl px-3 py-2 ${
              path.startsWith(item.href)
                ? "bg-gradient-to-r from-pink-500 to-purple-500 text-white"
                : "hover:bg-white/5 text-white/70"
            }`}
          >
            {item.label}
          </Link>
        ))}
      </nav>
      <div className="p-4 text-sm opacity-70">⏻ Log Out</div>
    </aside>
  );
}
