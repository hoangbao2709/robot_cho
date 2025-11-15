"use client";

export default function HeaderControl({
  mode,
  onToggle,
}: {
  mode: "remote" | "fpv";
  onToggle: () => void;
}) {
  const isFPV = mode === "fpv";

  return (
    <>
      <header className="flex items-center justify-between">
        {/* Tiêu đề gradient dùng class toàn cục */}
        <h1
          className={`gradient-title select-none transition-all duration-300 ${
            isFPV ? "opacity-100" : "opacity-90"
          }`}
        >
          {isFPV ? "FPV CONTROL MODE" : "REMOTE CONTROL MODE"}
        </h1>

        {/* Nút toggle Remote / FPV */}
        <div className="flex items-center gap-3">
          <span className="text-sm opacity-70">Remote</span>
          <button
            onClick={onToggle}
            className={`w-11 h-7 rounded-full border border-violet-400/50 p-1 grid items-center transition-all duration-300 ${
              isFPV ? "bg-violet-500/40" : "bg-transparent"
            }`}
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
    </>
  );
}
