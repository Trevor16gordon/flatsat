/** Formatting for a technical readout: fixed widths, no surprises. */

/** Seconds since the run began, as +M:SS.mmm — the axis a mission is
 *  actually read on. Absolute wall time belongs in the header, not on
 *  every tick. */
export function elapsed(ns: number, originNs: number): string {
  const s = (ns - originNs) / 1e9;
  const sign = s < 0 ? '-' : '+';
  const abs = Math.abs(s);
  const m = Math.floor(abs / 60);
  const rem = abs - m * 60;
  return `${sign}${m}:${rem.toFixed(3).padStart(6, '0')}`;
}

/** Duration in the largest sensible unit. */
export function duration(ns: number): string {
  const s = ns / 1e9;
  if (s < 1) return `${(s * 1000).toFixed(0)} ms`;
  if (s < 60) return `${s.toFixed(2)} s`;
  return `${Math.floor(s / 60)}m ${(s % 60).toFixed(1)}s`;
}

/**
 * A number at a readable precision for its magnitude.
 *
 * Engineering data spans many decades — torques near 1e-3, wheel speeds
 * in the hundreds — so a fixed decimal count is wrong for something.
 */
export function value(v: number): string {
  if (!Number.isFinite(v)) return '—';
  if (v === 0) return '0';
  const mag = Math.abs(v);
  if (mag >= 1e5 || mag < 1e-3) return v.toExponential(3);
  if (mag >= 100) return v.toFixed(2);
  if (mag >= 1) return v.toFixed(4);
  return v.toFixed(6);
}

export function wallClock(ns: number): string {
  if (!ns) return '—';
  return new Date(ns / 1e6).toISOString().replace('T', ' ').replace('Z', ' UTC');
}

/** Last path element of a channel key, for a compact legend. */
export function shortChannel(key: string): string {
  const dot = key.lastIndexOf('.');
  return dot < 0 ? key : key.slice(dot + 1);
}
