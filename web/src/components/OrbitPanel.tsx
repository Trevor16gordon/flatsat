/**
 * The orbit view: where the vehicle IS, drawn from truth telemetry.
 *
 * Everything here derives from the recorded `sim/truth/state` series —
 * no orbit propagation happens in the browser, so the view can never
 * disagree with what the run actually flew. The track is projected
 * onto the ORBIT PLANE (computed from two recorded position samples),
 * which turns a 3D ellipse into the circle it really is; eclipse arcs
 * are shaded, and at the scrub time the panel draws the body +z axis
 * and the sun direction rotated into inertial by the recorded truth
 * attitude — "is the panel face on the sun" at a glance.
 */

import { useEffect, useMemo, useRef, useState } from 'react';
import type { MissionBlob } from '../data/types';
import { elapsed } from '../lib/format';

const R_EARTH_M = 6378136.6;

/** Find the truth topic: any topic carrying position_x_m, preferring
 *  the one with the most channels (a scenario's scoped truth beats a
 *  leftover bridge topic recorded in the same window). */
function truthTopic(series: MissionBlob['series']): string | null {
  const counts = new Map<string, number>();
  for (const key of Object.keys(series)) {
    const dot_ = key.lastIndexOf('.');
    if (dot_ < 0) continue;
    const topic = key.slice(0, dot_);
    counts.set(topic, (counts.get(topic) ?? 0) + 1);
  }
  let best: string | null = null;
  for (const [topic, count] of counts) {
    if (!series[`${topic}.position_x_m`]) continue;
    if (best === null || count > (counts.get(best) ?? 0)) best = topic;
  }
  return best;
}

interface Props {
  blob: MissionBlob;
  originNs: number;
  /** The shared time window; the scrubber runs across it. */
  window: [number, number];
}

type Vec3 = [number, number, number];

/** MRP (sigma_BN) -> direction cosine matrix [BN], body from inertial. */
function mrpToDcm(s: Vec3): [Vec3, Vec3, Vec3] {
  const s2 = s[0] * s[0] + s[1] * s[1] + s[2] * s[2];
  const d = (1 + s2) * (1 + s2);
  const row = (a: number, b: number, c: number): Vec3 => [a / d, b / d, c / d];
  return [
    row(
      4 * (s[0] * s[0] - s[1] * s[1] - s[2] * s[2]) + (1 - s2) * (1 - s2),
      8 * s[0] * s[1] + 4 * s[2] * (1 - s2),
      8 * s[0] * s[2] - 4 * s[1] * (1 - s2),
    ),
    row(
      8 * s[1] * s[0] - 4 * s[2] * (1 - s2),
      4 * (-s[0] * s[0] + s[1] * s[1] - s[2] * s[2]) + (1 - s2) * (1 - s2),
      8 * s[1] * s[2] + 4 * s[0] * (1 - s2),
    ),
    row(
      8 * s[2] * s[0] + 4 * s[1] * (1 - s2),
      8 * s[2] * s[1] - 4 * s[0] * (1 - s2),
      4 * (-s[0] * s[0] - s[1] * s[1] + s[2] * s[2]) + (1 - s2) * (1 - s2),
    ),
  ];
}

const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a: Vec3) => Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
const unit = (a: Vec3): Vec3 => {
  const n = norm(a) || 1;
  return [a[0] / n, a[1] / n, a[2] / n];
};
const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

/** Index of the sample nearest to `t` (times ascending). */
function nearestIndex(times: number[], t: number): number {
  let lo = 0;
  let hi = times.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if ((times[mid] as number) <= t) lo = mid;
    else hi = mid;
  }
  return t - (times[lo] as number) <= (times[hi] as number) - t ? lo : hi;
}

export function OrbitPanel({ blob, originNs, window: win }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [frac, setFrac] = useState(1.0); // scrub position within the window

  // Pull the truth series once per blob. Positions are required; the
  // rest degrade gracefully (no attitude -> no arrows).
  const truth = useMemo(() => {
    const topic = truthTopic(blob.series);
    if (!topic) return null;
    const chan = (name: string) => blob.series[`${topic}.${name}`];
    const px = chan('position_x_m');
    const py = chan('position_y_m');
    const pz = chan('position_z_m');
    if (!px || !py || !pz || px.t_ns.length < 8) return null;
    return {
      t: px.t_ns,
      px: px.v,
      py: py.v,
      pz: pz.v,
      eclipse: chan('in_eclipse'),
      sigma: [chan('sigma_x'), chan('sigma_y'), chan('sigma_z')],
      sun: [chan('sun_x'), chan('sun_y'), chan('sun_z')],
    };
  }, [blob]);

  const cursorNs = win[0] + frac * (win[1] - win[0]);

  // Redraw on container resize; the first mount can measure 0x0 before
  // layout, and drawing then would ask for a negative Earth radius.
  const [sizeTick, setSizeTick] = useState(0);
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const observer = new ResizeObserver(() => setSizeTick((t) => t + 1));
    observer.observe(canvas);
    return () => observer.disconnect();
  }, [truth]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !truth) return;
    const dpr = globalThis.devicePixelRatio || 1;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w < 40 || h < 40) return; // not laid out yet; the observer retries
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, w, h);

    const n = truth.t.length;
    const at = (i: number): Vec3 => [
      truth.px[i] as number,
      truth.py[i] as number,
      truth.pz[i] as number,
    ];

    // Orbit-plane basis from two well-separated samples: e1 along the
    // first position, e2 completing the in-plane frame.
    const r0 = at(0);
    const r1 = at(Math.floor(n / 4) || 1);
    const planeNormal = unit(cross(r0, r1));
    const e1 = unit(r0);
    const e2 = unit(cross(planeNormal, e1));
    const project = (p: Vec3): [number, number] => [dot(p, e1), dot(p, e2)];

    let maxR = R_EARTH_M;
    for (let i = 0; i < n; i++) maxR = Math.max(maxR, norm(at(i)));
    const scale = (Math.min(w, h) / 2 - 14) / maxR;
    const cx = w / 2;
    const cy = h / 2;
    const toPx = (p: Vec3): [number, number] => {
      const [u, v] = project(p);
      return [cx + u * scale, cy - v * scale];
    };

    // Earth, with the terminator's shadow side implied by eclipse arcs.
    ctx.beginPath();
    ctx.arc(cx, cy, R_EARTH_M * scale, 0, 2 * Math.PI);
    ctx.fillStyle = 'rgba(64, 120, 192, 0.25)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(120, 170, 230, 0.8)';
    ctx.stroke();

    // The track: sunlit in theme gray, eclipse arcs darker and dashed.
    const eclipseAt = (i: number) =>
      truth.eclipse ? (truth.eclipse.v[i] as number) > 0.5 : false;
    for (let dark = 0; dark < 2; dark++) {
      ctx.beginPath();
      let pen = false;
      for (let i = 0; i < n; i++) {
        if (eclipseAt(i) !== (dark === 1)) {
          pen = false;
          continue;
        }
        const [x, y] = toPx(at(i));
        if (pen) ctx.lineTo(x, y);
        else ctx.moveTo(x, y);
        pen = true;
      }
      ctx.setLineDash(dark ? [3, 3] : []);
      ctx.strokeStyle = dark ? 'rgba(140, 140, 160, 0.7)' : 'rgba(220, 220, 230, 0.85)';
      ctx.lineWidth = 1.2;
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // The vehicle at the scrub time, with attitude if the run has it.
    const i = nearestIndex(truth.t, cursorNs);
    const pos = at(i);
    const [mx, my] = toPx(pos);
    ctx.beginPath();
    ctx.arc(mx, my, 4, 0, 2 * Math.PI);
    ctx.fillStyle = eclipseAt(i) ? '#8890a8' : '#ffd75e';
    ctx.fill();
    ctx.strokeStyle = '#111';
    ctx.stroke();

    const [sx, sy, sz] = truth.sigma;
    const arrow = (dir: Vec3, len: number, color: string, label: string) => {
      const tip: Vec3 = [pos[0] + dir[0] * len, pos[1] + dir[1] * len, pos[2] + dir[2] * len];
      const [tx, ty] = toPx(tip);
      ctx.beginPath();
      ctx.moveTo(mx, my);
      ctx.lineTo(tx, ty);
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.6;
      ctx.stroke();
      ctx.fillStyle = color;
      ctx.font = '10px ui-monospace, monospace';
      ctx.fillText(label, tx + 3, ty);
    };
    if (sx && sy && sz) {
      const sigma: Vec3 = [sx.v[i] as number, sy.v[i] as number, sz.v[i] as number];
      const bn = mrpToDcm(sigma);
      // Body +z in inertial is the third COLUMN of [NB] = row of [BN].
      const zInertial: Vec3 = [bn[2][0], bn[2][1], bn[2][2]];
      arrow(zInertial, maxR * 0.35, '#6ee7a8', '+z');
      const [ux, uy, uz] = truth.sun;
      if (ux && uy && uz && !eclipseAt(i)) {
        const sunBody: Vec3 = [ux.v[i] as number, uy.v[i] as number, uz.v[i] as number];
        if (norm(sunBody) > 0.5) {
          // s_N = [BN]^T s_B — the sun the truth says the body saw.
          const sunInertial: Vec3 = [
            bn[0][0] * sunBody[0] + bn[1][0] * sunBody[1] + bn[2][0] * sunBody[2],
            bn[0][1] * sunBody[0] + bn[1][1] * sunBody[1] + bn[2][1] * sunBody[2],
            bn[0][2] * sunBody[0] + bn[1][2] * sunBody[1] + bn[2][2] * sunBody[2],
          ];
          arrow(unit(sunInertial), maxR * 0.5, '#ffd75e', 'sun');
        }
      }
    }
  }, [truth, cursorNs, sizeTick]);

  if (!truth) return null;

  const i = nearestIndex(truth.t, cursorNs);
  const altKm =
    (norm([truth.px[i] as number, truth.py[i] as number, truth.pz[i] as number]) - R_EARTH_M) /
    1000;
  const dark = truth.eclipse ? (truth.eclipse.v[i] as number) > 0.5 : false;

  return (
    <section className="panel orbit-panel">
      <div className="panel-head">
        <span>orbit</span>
        <span className="panel-note">
          {elapsed(cursorNs, originNs)} · alt {altKm.toFixed(0)} km ·{' '}
          {dark ? 'eclipse' : 'sunlit'}
        </span>
      </div>
      <div className="orbit-body">
        <canvas ref={canvasRef} className="orbit-canvas" />
        <input
          className="orbit-scrub"
          type="range"
          min={0}
          max={1000}
          value={Math.round(frac * 1000)}
          onChange={(e) => setFrac(Number(e.target.value) / 1000)}
          title="scrub the marker through the visible time window"
        />
      </div>
    </section>
  );
}
