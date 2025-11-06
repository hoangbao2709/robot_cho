import ConnectionPage from "./(dashboard)/connection/page";
import Page_remotr from "./page/Page_remotr";
import Sidebar from "@/components/Sidebar";
import Topbar from "@/components/Topbar";
export default function Home() {
  return (
      <div className="min-h-screen bg-[#1A0F28] text-white">
            <div className="flex min-h-screen">
              {/* Sidebar bên trái */}
              <Sidebar />
              {/* Phần chính bên phải */}
              <div className="flex flex-col flex-1">
                <Topbar />
                <main className="flex-1 p-6">{<Page_remotr />}</main>
              </div>
            </div>
          </div>
  );
}
