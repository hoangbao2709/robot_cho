"use client";

type Device = {
  id: number;
  name: string;
  ip: string;
  battery: number;
  status: "online" | "offline" | "unknown";
};

export default function ConnectionCard({
  device,
  onConnect,
}: {
  device: Device;
  onConnect?: (device: Device) => void;
}) {
  return (
    <div className="flex justify-between items-center rounded-2xl bg-white/10 p-4">
      <div>
        <div className="font-semibold flex items-center gap-2">
          <span>{device.name}</span>
          <span
            className={`text-xs px-2 py-0.5 rounded-full ${
              device.status === "online"
                ? "bg-green-500/20 text-green-300"
                : device.status === "offline"
                ? "bg-rose-500/20 text-rose-300"
                : "bg-yellow-500/20 text-yellow-300"
            }`}
          >
            {device.status}
          </span>
        </div>
        <div className="text-sm text-white/70">IP: {device.ip}</div>
        <div className="text-sm text-white/70">Battery: {device.battery}%</div>
      </div>
      <button
        onClick={() => onConnect && onConnect(device)}
        className="rounded-xl bg-gradient-to-r from-pink-500 to-purple-500 px-4 py-2 text-sm cursor-pointer"
      >
        Connect
      </button>
    </div>
  );
}

