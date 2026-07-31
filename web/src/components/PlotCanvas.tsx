/**
 * The plot: boxed axes, fine grid, drag to zoom, crosshair readout.
 *
 * Canvas rather than SVG. A mission channel can hold thousands of
 * points and a handful of channels are drawn at once; SVG would put
 * tens of thousands of nodes in the DOM and make pan sluggish, while a
 * canvas redraw at this size is comfortably under a frame.
 *
 * Two things are deliberately old-fashioned because they are right: the
 * axes are a closed box with inward ticks, and the grid is faint rather
 * than absent. Engineering plots are read by tracing a value across to
 * an axis, and that needs a reference.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import type { Series } from '../data/types';
import { elapsed, shortChannel, value as fmtValue } from '../lib/format';
import { linear, padded, ticks } from '../lib/scale';

export interface PlotChannel {
  key: string;
  series: Series;
  color: string;
}

interface Props {
  channels: PlotChannel[];
  originNs: number;
  /** Visible window; null fits all data. */
  viewNs: [number, number] | null;
  onViewChange: (view: [number, number] | null) => void;
  /** Vertical marks: annotations and span edges. */
  marks: { timeNs: number; color: string; label: string }[];
}

const PAD = { left: 78, right: 18, top: 14, bottom: 34 };

export function PlotCanvas({ channels, originNs, viewNs, onViewChange, marks }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const wrapRef = useRef<HTMLDivElement | null>(null);
  const [size, setSize] = useState({ w: 800, h: 420 });
  const [cursor, setCursor] = useState<{ x: number; y: number } | null>(null);
  const [drag, setDrag] = useState<{ from: number; to: number } | null>(null);

  // Domain over the data, then over the visible window.
  const domain = useMemo(() => {
    let t0 = Infinity;
    let t1 = -Infinity;
    let v0 = Infinity;
    let v1 = -Infinity;
    for (const ch of channels) {
      const { t_ns, v } = ch.series;
      if (t_ns.length === 0) continue;
      t0 = Math.min(t0, t_ns[0] as number);
      t1 = Math.max(t1, t_ns[t_ns.length - 1] as number);
      for (let i = 0; i < v.length; i++) {
        const y = v[i] as number;
        if (y < v0) v0 = y;
        if (y > v1) v1 = y;
      }
    }
    if (!Number.isFinite(t0)) return null;
    return { t0, t1, v0, v1 };
  }, [channels]);

  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const ro = new ResizeObserver(() => {
      setSize({ w: el.clientWidth, h: el.clientHeight });
    });
    ro.observe(el);
    setSize({ w: el.clientWidth, h: el.clientHeight });
    return () => ro.disconnect();
  }, []);

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas || !domain) return;
    const dpr = window.devicePixelRatio || 1;
    const { w, h } = size;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const css = getComputedStyle(canvas);
    const ink = css.getPropertyValue('--plot-ink').trim() || '#1c2530';
    const grid = css.getPropertyValue('--plot-grid').trim() || '#dfe4ea';
    const face = css.getPropertyValue('--plot-face').trim() || '#ffffff';

    const [tl, tr] = viewNs ?? [domain.t0, domain.t1];
    const [vl, vh] = padded(domain.v0, domain.v1);
    const x = linear(tl, tr, PAD.left, w - PAD.right);
    const y = linear(vl, vh, h - PAD.bottom, PAD.top);

    // Plot face.
    ctx.fillStyle = face;
    ctx.fillRect(PAD.left, PAD.top, w - PAD.left - PAD.right, h - PAD.top - PAD.bottom);

    // Grid and tick labels.
    ctx.font = '11px ui-monospace, "SF Mono", Menlo, monospace';
    ctx.strokeStyle = grid;
    ctx.fillStyle = ink;
    ctx.lineWidth = 1;
    for (const t of ticks(vl, vh, 6)) {
      const py = Math.round(y(t)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(PAD.left, py);
      ctx.lineTo(w - PAD.right, py);
      ctx.stroke();
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      ctx.fillText(fmtValue(t), PAD.left - 8, py);
    }
    for (const t of ticks(tl, tr, 7)) {
      const px = Math.round(x(t)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(px, PAD.top);
      ctx.lineTo(px, h - PAD.bottom);
      ctx.stroke();
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillText(elapsed(t, originNs), px, h - PAD.bottom + 7);
    }

    // Event marks, behind the traces so data stays legible.
    for (const m of marks) {
      if (m.timeNs < tl || m.timeNs > tr) continue;
      const px = Math.round(x(m.timeNs)) + 0.5;
      ctx.save();
      ctx.strokeStyle = m.color;
      ctx.globalAlpha = 0.5;
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.moveTo(px, PAD.top);
      ctx.lineTo(px, h - PAD.bottom);
      ctx.stroke();
      ctx.restore();
    }

    // Traces.
    ctx.lineWidth = 1.4;
    ctx.lineJoin = 'round';
    for (const ch of channels) {
      const { t_ns, v } = ch.series;
      ctx.strokeStyle = ch.color;
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < t_ns.length; i++) {
        const t = t_ns[i] as number;
        if (t < tl || t > tr) {
          started = false; // leave the window rather than draw across it
          continue;
        }
        const px = x(t);
        const py = y(v[i] as number);
        if (!started) {
          ctx.moveTo(px, py);
          started = true;
        } else {
          ctx.lineTo(px, py);
        }
      }
      ctx.stroke();
    }

    // Axis box last, so traces never overhang it.
    ctx.strokeStyle = ink;
    ctx.lineWidth = 1;
    ctx.strokeRect(
      PAD.left + 0.5,
      PAD.top + 0.5,
      w - PAD.left - PAD.right - 1,
      h - PAD.top - PAD.bottom - 1,
    );

    // Zoom selection.
    if (drag) {
      const a = Math.min(drag.from, drag.to);
      const b = Math.max(drag.from, drag.to);
      ctx.fillStyle = 'rgba(0,114,189,0.14)';
      ctx.fillRect(a, PAD.top, b - a, h - PAD.top - PAD.bottom);
      ctx.strokeStyle = '#0072BD';
      ctx.strokeRect(a + 0.5, PAD.top + 0.5, b - a, h - PAD.top - PAD.bottom - 1);
    }

    // Crosshair.
    if (cursor && !drag) {
      ctx.save();
      ctx.strokeStyle = ink;
      ctx.globalAlpha = 0.35;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(Math.round(cursor.x) + 0.5, PAD.top);
      ctx.lineTo(Math.round(cursor.x) + 0.5, h - PAD.bottom);
      ctx.stroke();
      ctx.restore();
    }
  }, [channels, cursor, domain, drag, marks, originNs, size, viewNs]);

  useEffect(() => {
    draw();
  }, [draw]);

  /** Canvas-relative pixel x -> absolute time. */
  const toTime = (localX: number): number => {
    if (!domain) return 0;
    const [tl, tr] = viewNs ?? [domain.t0, domain.t1];
    return linear(tl, tr, PAD.left, size.w - PAD.right).invert(localX);
  };

  // Value of each channel at the cursor — the data-cursor readout that
  // makes a plot interrogable rather than decorative.
  const readout = useMemo(() => {
    if (!cursor || !domain) return null;
    const tAt = toTime(cursor.x);
    return {
      timeNs: tAt,
      rows: channels.map((ch) => {
        const { t_ns, v } = ch.series;
        let lo = 0;
        let hi = t_ns.length - 1;
        while (lo < hi) {
          const mid = (lo + hi) >> 1;
          if ((t_ns[mid] as number) < tAt) lo = mid + 1;
          else hi = mid;
        }
        return { key: ch.key, color: ch.color, v: v[lo] };
      }),
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cursor, channels, domain, viewNs, size]);

  if (!domain) {
    return (
      <div ref={wrapRef} className="plot empty">
        <span>select one or more channels</span>
      </div>
    );
  }

  return (
    <div ref={wrapRef} className="plot">
      <canvas
        ref={canvasRef}
        style={{ width: '100%', height: '100%' }}
        onMouseMove={(e) => {
          const rect = e.currentTarget.getBoundingClientRect();
          const x = e.clientX - rect.left;
          setCursor({ x, y: e.clientY - rect.top });
          if (drag) setDrag({ ...drag, to: x });
        }}
        onMouseLeave={() => {
          setCursor(null);
          setDrag(null);
        }}
        onMouseDown={(e) => {
          const rect = e.currentTarget.getBoundingClientRect();
          const x = e.clientX - rect.left;
          setDrag({ from: x, to: x });
        }}
        onMouseUp={() => {
          if (!drag) return;
          const a = Math.min(drag.from, drag.to);
          const b = Math.max(drag.from, drag.to);
          setDrag(null);
          if (b - a < 6) return; // a click, not a selection
          onViewChange([toTime(a), toTime(b)]);
        }}
        onDoubleClick={() => onViewChange(null)}
      />
      {readout && (
        <div className="readout">
          <div className="readout-time">{elapsed(readout.timeNs, originNs)}</div>
          {readout.rows.map((r) => (
            <div key={r.key} className="readout-row">
              <span className="swatch" style={{ background: r.color }} />
              <span className="readout-name">{shortChannel(r.key)}</span>
              <span className="readout-value">{fmtValue(r.v ?? NaN)}</span>
            </div>
          ))}
        </div>
      )}
      <div className="plot-hint">drag to zoom · double-click to reset</div>
    </div>
  );
}
