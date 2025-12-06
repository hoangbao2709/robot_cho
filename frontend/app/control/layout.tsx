import React, { Suspense } from "react";

export default function ControlLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <Suspense fallback={<div className="text-white p-6">Loading control...</div>}>
      {children}
    </Suspense>
  );
}
