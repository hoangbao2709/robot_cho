
import Sidebar from "@/components/Sidebar";
import Topbar from "@/components/Topbar";

export default function DashboardLayout({ children }: { children: React.ReactNode }) {
  return (
      <div className="min-h-screen bg-[#1A0F28] text-white">
        <div className="flex min-h-screen">
          {/* Sidebar bên trái */}
          <Sidebar />
          {/* Phần chính bên phải */}
          <div className="flex flex-col flex-1">
            <Topbar />
            <main className="flex-1 p-6">{children}</main>
          </div>
        </div>
      </div>
  );
}
