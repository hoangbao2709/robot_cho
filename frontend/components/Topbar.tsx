export default function Topbar() {
  return (
    <header className="border-b border-white/10 bg-[#1A0F28]/70 backdrop-blur p-4 flex justify-between">
      <input
        placeholder="Search..."
        className="rounded-full bg-white/10 px-4 py-2 text-sm placeholder:text-white/60 focus:outline-none flex-1 max-w-md"
      />
      <div className="flex items-center gap-3 text-sm text-white/80">
        <span>Ramon Ridwan</span>
        <div className="h-8 w-8 rounded-full bg-white/20"></div>
      </div>
    </header>
  );
}
