// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0.

import {
  useEffect,
  useId,
  useMemo,
  useState,
} from "react";

const DEGREES_PER_RADIAN = 180 / Math.PI;

export function normalizeYawDegrees(value) {
  if (typeof value === "string" && !value.trim()) return null;
  const degrees = Number(value);
  if (!Number.isFinite(degrees)) return null;
  const normalized = ((degrees + 180) % 360 + 360) % 360 - 180;
  return Object.is(normalized, -0) ? 0 : normalized;
}

export function yawDegreesToRadians(value) {
  const degrees = normalizeYawDegrees(value);
  return degrees === null ? null : degrees / DEGREES_PER_RADIAN;
}

export function yawRadiansToDegrees(value) {
  const radians = Number(value);
  if (!Number.isFinite(radians)) return 0;
  return normalizeYawDegrees(radians * DEGREES_PER_RADIAN) ?? 0;
}

function formatDegrees(value) {
  return Number(value.toFixed(3)).toString();
}

export default function WaypointYawEditor({
  disabled = false,
  label,
  yaw = 0,
  onApply,
}) {
  const inputId = useId();
  const currentDegrees = useMemo(() => yawRadiansToDegrees(yaw), [yaw]);
  const [draft, setDraft] = useState(() => formatDegrees(currentDegrees));
  const [saving, setSaving] = useState(false);
  const parsedDegrees = normalizeYawDegrees(draft);
  const valid = parsedDegrees !== null;
  const changed = valid && Math.abs(parsedDegrees - currentDegrees) > 0.0005;

  useEffect(() => {
    setDraft(formatDegrees(currentDegrees));
  }, [currentDegrees]);

  const submit = async (event) => {
    event.preventDefault();
    const radians = yawDegreesToRadians(draft);
    if (disabled || saving || radians === null || !changed) return;
    setSaving(true);
    try {
      await onApply(radians);
      setDraft(formatDegrees(normalizeYawDegrees(draft)));
    } finally {
      setSaving(false);
    }
  };

  return (
    <form onSubmit={submit} className="grid gap-1.5 px-1 pb-0.5">
      <div className="grid grid-cols-[58px_minmax(0,1fr)_52px] items-center gap-1.5">
        <label
          htmlFor={inputId}
          className="text-[10.5px] font-semibold"
          style={{ color: "var(--mc-text-muted)" }}
        >
          Yaw (°)
        </label>
        <input
          id={inputId}
          aria-label={`Yaw for ${label}`}
          type="number"
          step="0.1"
          inputMode="decimal"
          value={draft}
          disabled={disabled || saving}
          onChange={(event) => setDraft(event.currentTarget.value)}
          className="h-7 min-w-0 px-2 text-[11px] font-mono disabled:opacity-45"
          style={{
            borderRadius: 7,
            border: `1px solid ${valid ? "var(--mc-border-strong)" : "var(--mc-danger)"}`,
            backgroundColor: "var(--mc-surface)",
            color: "var(--mc-text)",
          }}
        />
        <button
          type="submit"
          disabled={disabled || saving || !changed}
          className="h-7 px-2 text-[10.5px] font-semibold active:translate-y-px disabled:opacity-40"
          style={{
            borderRadius: 7,
            border: "1px solid var(--mc-border-strong)",
            backgroundColor: "var(--mc-surface)",
            color: "var(--mc-text)",
          }}
        >
          {saving ? "Saving" : "Apply"}
        </button>
      </div>
      <div className="text-[9.5px] font-mono" style={{ color: "var(--mc-text-subtle)" }}>
        Current: {Number(yaw).toFixed(4)} rad · values normalize to −180°…180°
      </div>
    </form>
  );
}
