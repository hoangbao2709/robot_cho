"use client";
import { useState } from "react";

export default function LoginPage() {
  const [showPassword, setShowPassword] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  async function onSubmit(e: React.FormEvent<HTMLFormElement>) {
    e.preventDefault();
    const data = new FormData(e.currentTarget);
    const email = String(data.get("email") || "").trim();
    const password = String(data.get("password") || "");

    setError(null);
    if (!email || !password) {
      setError("Please enter email and password.");
      return;
    }
    setLoading(true);
    await new Promise((r) => setTimeout(r, 900));
    setLoading(false);
    alert(`Logged in as ${email}`);
  }

  return (
    <main className="min-h-screen bg-[#0c0520] text-white grid place-items-center p-6">
      <div className="w-full max-w-md">
        <div className="relative rounded-3xl border border-white/10 bg-white/5 p-6 sm:p-8 shadow-[0_0_0_1px_rgba(255,255,255,0.02)]">
          <div className="pointer-events-none absolute inset-0 rounded-3xl ring-1 ring-inset ring-fuchsia-500/20"></div>
          <div className="mb-6 text-center">
            <div className="inline-flex items-center gap-2">
              <span className="h-3 w-3 rounded-full bg-pink-400 shadow-[0_0_20px_2px_rgba(244,114,182,0.6)]"></span>
              <h1 className="text-2xl font-bold tracking-tight">
                <span className="text-pink-400">ROBOT</span>{" "}
                <span className="text-sky-300">CONTROL</span>{" "}
                <span className="text-indigo-400">LOGIN</span>
              </h1>
            </div>
            <p className="mt-2 text-sm text-white/70">Sign in to continue to the Remote Control Mode.</p>
          </div>

          <form onSubmit={onSubmit} className="space-y-5">
            <FieldLabel htmlFor="email">Email</FieldLabel>
            <div className="relative">
              <input
                id="email"
                name="email"
                type="email"
                placeholder="you@example.com"
                className="w-full rounded-xl bg-white/5 border border-white/10 px-4 py-3 outline-none placeholder:text-white/40 focus:border-fuchsia-400/50 focus:ring-2 focus:ring-fuchsia-400/30"
                autoComplete="email"
              />
              <div className="pointer-events-none absolute inset-0 rounded-xl ring-1 ring-inset ring-white/5"></div>
            </div>

            <FieldLabel htmlFor="password">Password</FieldLabel>
            <div className="relative">
              <input
                id="password"
                name="password"
                type={showPassword ? "text" : "password"}
                placeholder="••••••••"
                className="w-full rounded-xl bg-white/5 border border-white/10 px-4 py-3 pr-12 outline-none placeholder:text-white/40 focus:border-indigo-400/50 focus:ring-2 focus:ring-indigo-400/30"
                autoComplete="current-password"
              />
              <button
                type="button"
                onClick={() => setShowPassword((s) => !s)}
                className="absolute right-2 top-1/2 -translate-y-1/2 rounded-lg px-3 py-1 text-xs bg-white/5 border border-white/10 hover:bg-white/10"
                aria-label={showPassword ? "Hide password" : "Show password"}
              >
                {showPassword ? "Hide" : "Show"}
              </button>
            </div>

            <div className="flex items-center justify-between text-sm">
              <label className="inline-flex items-center gap-2 select-none">
                <input type="checkbox" name="remember" className="accent-fuchsia-400" />
                <span className="text-white/80">Remember me</span>
              </label>
              <a href="#" className="text-sky-300 hover:text-sky-200">Forgot password?</a>
            </div>

            {error && (
              <p className="text-sm text-rose-300 bg-rose-500/10 border border-rose-400/30 rounded-lg px-3 py-2">{error}</p>
            )}

            <button
              type="submit"
              disabled={loading}
              className="w-full rounded-xl border border-fuchsia-400/40 bg-gradient-to-r from-pink-500/30 via-indigo-500/30 to-sky-500/30 px-4 py-3 font-semibold hover:from-pink-500/40 hover:via-indigo-500/40 hover:to-sky-500/40 disabled:opacity-60"
            >
              {loading ? "Signing in..." : "Sign in"}
            </button>

            <div className="relative py-3 text-center">
              <div className="border-t border-white/10" />
              <span className="absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 bg-white/5 px-3 text-xs text-white/60">or</span>
            </div>

            <div className="grid grid-cols-2 gap-3">
              <button type="button" className="rounded-xl bg-white/5 border border-white/10 px-4 py-2 hover:bg-white/10">Login with Google</button>
              <button type="button" className="rounded-xl bg-white/5 border border-white/10 px-4 py-2 hover:bg-white/10">Login with GitHub</button>
            </div>
          </form>
        </div>

        <p className="mt-6 text-center text-xs text-white/50">
          By continuing, you agree to our <a href="#" className="text-sky-300 hover:text-sky-200">Terms</a> and <a href="#" className="text-sky-300 hover:text-sky-200">Privacy</a>.
        </p>
      </div>
    </main>
  );
}

function FieldLabel({ children, htmlFor }: { children: React.ReactNode; htmlFor: string }) {
  return (
    <label htmlFor={htmlFor} className="block text-sm font-medium text-white/80">
      {children}
    </label>
  );
}
