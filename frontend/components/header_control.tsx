"use client";

type HeaderControlProps = {
  mode: "remote" | "fpv";
  onToggle: () => void;
  // Nếu sau này muốn xử lý nút Disconnect thì thêm:
  // onDisconnect?: () => void;
};

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-xl bg-white/5 border border-white/10 p-3">
      <div className="text-[10px] uppercase opacity-60">{label}</div>
      <div className="text-sm mt-1">{value}</div>
    </div>
  );
}

export default function HeaderControl({ mode, onToggle }: HeaderControlProps) {
  const isFPV = mode === "fpv";

  return (
    <>
      {/* Header trên cùng: tiêu đề + toggle Remote / FPV + nút Disconnect */}
      <header className="flex items-center justify-between">
        {/* Tiêu đề gradient dùng class global */}
        <h1
          className={`gradient-title select-none transition-all duration-300 ${
            isFPV ? "opacity-100" : "opacity-90"
          }`}
        >
          {isFPV ? "FPV CONTROL MODE" : "REMOTE CONTROL MODE"}
        </h1>

        {/* Nhóm nút bên phải */}
        <div className="flex items-center gap-3">
          <button className="px-3 py-1 rounded-xl bg-pink-500/20 hover:bg-pink-500/30 text-pink-300 text-sm">
            Disconnect
          </button>

          <span className="text-sm opacity-70">Remote</span>
          {/* Toggle giống bản 1 nhưng dùng state mode từ props */}
          <button
            onClick={onToggle}
            className={`w-11 h-7 rounded-full border border-violet-400/50 p-1 grid items-center transition-all duration-300 ${
              isFPV ? "bg-violet-500/40" : "bg-transparent"
            }`}
            aria-label="Toggle FPV mode"
          >
            <div
              className={`w-5 h-5 rounded-full border border-violet-300/60 bg-white/10 transition-transform duration-300 ${
                isFPV ? "translate-x-4" : ""
              }`}
            />
          </button>
          <span className="text-sm">FPV</span>
        </div>
      </header>

      {/* Hàng dưới: Robot info + metrics + lidar map */}
      <div className="mt-4 grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Robot A + metrics */}
        <div className="col-span-2 p-4 rounded-2xl bg-white/5 border border-white/10">
          <div className="flex flex-wrap items-center gap-6">
            <div className="text-lg font-semibold">🤖 Robot A</div>
            <div className="text-xs uppercase opacity-60">Robot Details</div>
          </div>
          <div className="mt-4 grid grid-cols-3 gap-4 text-sm">
            <Metric label="Location" value="25.23234, 19.76543" />
            <Metric label="Cleaning Progress" value="80% (stopped)" />
            <Metric label="Floor" value="1st" />
            <Metric label="Status" value="Resting" />
            <Metric label="Water Level" value="50%" />
            <Metric label="Battery" value="85%" />
          </div>
        </div>
      </div>
    </>
  );
}
