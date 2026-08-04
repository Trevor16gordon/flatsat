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
 *
 * Zoom is modal on the drag: a plain drag selects a TIME range, a
 * ⌘-drag (ctrl elsewhere) selects a VALUE range. Double-click resets
 * both. Time labels are drawn at 45° — horizontal elapsed-time labels
 * collide long before the ticks stop being useful.
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
  /** Visible time window; null fits all data. */
  viewNs: [number, number] | null;
  /** Visible value window; null fits the channels. */
  viewV: [number, number] | null;
  onViewChange: (view: [number, number] | null) => void;
  onViewVChange: (view: [number, number] | null) => void;
  /** Vertical marks: annotations and span edges. */
  marks: { timeNs: number; color: string; label: string }[];
}

const PAD = { left: 78, right: 18, top: 14, bottom: 52 };

type DragAxis = 'x' | 'y';

export function PlotCanvas({
  channels,
  originNs,
  viewNs,
  viewV,
  onViewChange,
  onViewVChange,
  marks,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const wrapRef = useRef<HTMLDivElement | null>(null);
  const [size, setSize] = useState({ w: 800, h: 420 });
  const [cursor, setCursor] = useState<{ x: number; y: number } | null>(null);
  const [drag, setDrag] = useState<{ axis: DragAxis; from: number; to: number } | null>(null);

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

  const valueRange = useCallback((): [number, number] => {
    if (viewV) return viewV;
    if (!domain) return [0, 1];
    return padded(domain.v0, domain.v1);
  }, [domain, viewV]);

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
    const [vl, vh] = valueRange();
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
      if (py < PAD.top || py > h - PAD.bottom) continue;
      ctx.beginPath();
      ctx.moveTo(PAD.left, py);
      ctx.lineTo(w - PAD.right, py);
      ctx.stroke();
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      ctx.fillText(fmtValue(t), PAD.left - 8, py);
    }
    // Rotated labels overlap far later than horizontal ones, so ticks
    // can sit denser than the old 90 px budget.
    const xTarget = Math.max(3, Math.floor((w - PAD.left - PAD.right) / 64));
    for (const t of ticks(tl, tr, xTarget)) {
      const px = Math.round(x(t)) + 0.5;
      if (px < PAD.left || px > w - PAD.right) continue;
      ctx.beginPath();
      ctx.moveTo(px, PAD.top);
      ctx.lineTo(px, h - PAD.bottom);
      ctx.stroke();
      // 45° labels, anchored at their tick, reading up toward it.
      ctx.save();
      ctx.translate(px, h - PAD.bottom + 8);
      ctx.rotate(-Math.PI / 4);
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      ctx.fillText(elapsed(t, originNs), 0, 0);
      ctx.restore();
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

    // Traces, clipped to the face: a value zoom must crop, not overhang.
    ctx.save();
    ctx.beginPath();
    ctx.rect(PAD.left, PAD.top, w - PAD.left - PAD.right, h - PAD.top - PAD.bottom);
    ctx.clip();
    ctx.lineWidth = 1.4;
    ctx.lineJoin = 'round';
    for (const ch of channels) {
      const { t_ns, v } = ch.series;
      ctx.strokeStyle = ch.color;
      ctx.beginPath();
      let started = false;
      let runLen = 0;
      let last: [number, number] | null = null;
      // A run of one sample strokes nothing — mark it with a dot instead,
      // so sparse channels (mode changes, single events) stay visible.
      const lone: [number, number][] = [];
      for (let i = 0; i < t_ns.length; i++) {
        const t = t_ns[i] as number;
        if (t < tl || t > tr) {
          if (runLen === 1 && last) lone.push(last);
          started = false; // leave the window rather than draw across it
          runLen = 0;
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
        runLen += 1;
        last = [px, py];
      }
      if (runLen === 1 && last) lone.push(last);
      ctx.stroke();
      ctx.fillStyle = ch.color;
      for (const [px, py] of lone) {
        ctx.beginPath();
        ctx.arc(px, py, 2.5, 0, Math.PI * 2);
        ctx.fill();
      }
    }
    ctx.restore();

    // Axis box last, so traces never overhang it.
    ctx.strokeStyle = ink;
    ctx.lineWidth = 1;
    ctx.strokeRect(
      PAD.left + 0.5,
      PAD.top + 0.5,
      w - PAD.left - PAD.right - 1,
      h - PAD.top - PAD.bottom - 1,
    );

    // Zoom selection: a vertical band for a time range, a horizontal
    // band for a value range.
    if (drag) {
      const a = Math.min(drag.from, drag.to);
      const b = Math.max(drag.from, drag.to);
      ctx.fillStyle = 'rgba(0,114,189,0.14)';
      ctx.strokeStyle = '#0072BD';
      if (drag.axis === 'x') {
        ctx.fillRect(a, PAD.top, b - a, h - PAD.top - PAD.bottom);
        ctx.strokeRect(a + 0.5, PAD.top + 0.5, b - a, h - PAD.top - PAD.bottom - 1);
      } else {
        ctx.fillRect(PAD.left, a, w - PAD.left - PAD.right, b - a);
        ctx.strokeRect(PAD.left + 0.5, a + 0.5, w - PAD.left - PAD.right - 1, b - a);
      }
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
  }, [channels, cursor, domain, drag, marks, originNs, size, valueRange, viewNs]);

  useEffect(() => {
    draw();
  }, [draw]);

  // The canvas is a bitmap: an OS light/dark switch restyles the DOM but
  // leaves the last paint behind unless we repaint on the media change.
  useEffect(() => {
    const mq = window.matchMedia('(prefers-color-scheme: dark)');
    // Next frame, not synchronously: the new theme's CSS variables may not
    // have been recalculated when the media-query event fires.
    const onChange = () => requestAnimationFrame(draw);
    mq.addEventListener('change', onChange);
    return () => mq.removeEventListener('change', onChange);
  }, [draw]);

  /** Canvas-relative pixel x -> absolute time. */
  const toTime = (localX: number): number => {
    if (!domain) return 0;
    const [tl, tr] = viewNs ?? [domain.t0, domain.t1];
    return linear(tl, tr, PAD.left, size.w - PAD.right).invert(localX);
  };

  /** Canvas-relative pixel y -> value. */
  const toValue = (localY: number): number => {
    const [vl, vh] = valueRange();
    return linear(vl, vh, size.h - PAD.bottom, PAD.top).invert(localY);
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
        <span>drop channels here</span>
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
          const y = e.clientY - rect.top;
          setCursor({ x, y });
          if (drag) setDrag({ ...drag, to: drag.axis === 'x' ? x : y });
        }}
        onMouseLeave={() => {
          setCursor(null);
          setDrag(null);
        }}
        onMouseDown={(e) => {
          const rect = e.currentTarget.getBoundingClientRect();
          const axis: DragAxis = e.metaKey || e.ctrlKey ? 'y' : 'x';
          const at = axis === 'x' ? e.clientX - rect.left : e.clientY - rect.top;
          setDrag({ axis, from: at, to: at });
        }}
        onMouseUp={() => {
          if (!drag) return;
          const a = Math.min(drag.from, drag.to);
          const b = Math.max(drag.from, drag.to);
          setDrag(null);
          if (b - a < 6) return; // a click, not a selection
          if (drag.axis === 'x') {
            onViewChange([toTime(a), toTime(b)]);
          } else {
            // Screen y grows downward, value grows upward: the lower
            // pixel edge is the higher value.
            onViewVChange([toValue(b), toValue(a)]);
          }
        }}
        onDoubleClick={() => {
          onViewChange(null);
          onViewVChange(null);
        }}
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
      <div className="plot-hint">drag: zoom time · ⌘drag: zoom value · double-click: reset</div>
    </div>
  );
}
