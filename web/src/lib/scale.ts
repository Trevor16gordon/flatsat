/** Linear scales and tick generation — the arithmetic behind an axis. */

export interface Scale {
  (v: number): number;
  invert(px: number): number;
}

export function linear(d0: number, d1: number, r0: number, r1: number): Scale {
  const span = d1 - d0 || 1;
  const fn = ((v: number) => r0 + ((v - d0) / span) * (r1 - r0)) as Scale;
  fn.invert = (px: number) => d0 + ((px - r0) / (r1 - r0 || 1)) * span;
  return fn;
}

/**
 * Ticks at 1/2/5 x 10^n — the spacing that reads as "round" to a human.
 *
 * Naive division produces ticks at 0.037 intervals, which is correct and
 * unreadable.
 */
export function ticks(lo: number, hi: number, target = 6): number[] {
  if (!Number.isFinite(lo) || !Number.isFinite(hi) || lo === hi) return [lo];
  const raw = (hi - lo) / target;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const norm = raw / mag;
  const step = (norm >= 5 ? 5 : norm >= 2 ? 2 : 1) * mag;
  const out: number[] = [];
  for (let t = Math.ceil(lo / step) * step; t <= hi + step * 1e-9; t += step) {
    out.push(Math.abs(t) < step * 1e-9 ? 0 : t);
  }
  return out;
}

/** Pad a data range so traces do not touch the axis box. */
export function padded(lo: number, hi: number, frac = 0.06): [number, number] {
  if (lo === hi) {
    const bump = Math.abs(lo) * 0.1 || 1;
    return [lo - bump, hi + bump];
  }
  const pad = (hi - lo) * frac;
  return [lo - pad, hi + pad];
}
