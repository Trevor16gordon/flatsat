/**
 * The span swimlane: what the spacecraft was DOING, over the same axis
 * as the plot.
 *
 * Nesting is shown by indent, concurrency by overlap — a mission span
 * containing phases that may run alongside each other. An unclosed span
 * is drawn hatched to its right edge rather than omitted, because a run
 * that died mid-phase must not read like one that finished.
 */

import type { Annotation, Span } from '../data/types';
import { duration, elapsed } from '../lib/format';
import { levelColor, outcomeColor } from '../lib/palette';

interface Props {
  spans: Span[];
  annotations: Annotation[];
  originNs: number;
  window: [number, number];
  onSelectSpan: (span: Span) => void;
  selectedSpanId: string | null;
}

/** Depth of a span in the parent chain, for indentation. */
function depthOf(span: Span, byId: Map<string, Span>): number {
  let depth = 0;
  let cur = span;
  while (cur.parent_span_id && byId.has(cur.parent_span_id) && depth < 8) {
    cur = byId.get(cur.parent_span_id) as Span;
    depth += 1;
  }
  return depth;
}

export function Timeline({
  spans,
  annotations,
  originNs,
  window: win,
  onSelectSpan,
  selectedSpanId,
}: Props) {
  const [t0, t1] = win;
  const span = t1 - t0 || 1;
  const pct = (ns: number) => ((ns - t0) / span) * 100;
  const byId = new Map(spans.map((s) => [s.span_id, s]));

  if (spans.length === 0) {
    return <div className="timeline empty">no mission spans in this run</div>;
  }

  return (
    <div className="timeline">
      {spans.map((s) => {
        const start = s.start_ns ?? t0;
        const end = s.end_ns ?? t1;
        const depth = depthOf(s, byId);
        const open = s.end_ns === null;
        return (
          <div
            key={s.span_id}
            className={`lane${selectedSpanId === s.span_id ? ' selected' : ''}`}
            onClick={() => onSelectSpan(s)}
            title={s.detail || s.name}
          >
            <div className="lane-label" style={{ paddingLeft: 8 + depth * 14 }}>
              {s.name || '(unnamed)'}
            </div>
            <div className="lane-track">
              <div
                className={`bar${open ? ' open' : ''}`}
                style={{
                  left: `${pct(start)}%`,
                  width: `${Math.max(pct(end) - pct(start), 0.4)}%`,
                  background: outcomeColor(s.outcome),
                }}
              >
                <span className="bar-text">
                  {s.end_ns !== null ? duration(end - start) : 'never closed'}
                </span>
              </div>
              {annotations
                .filter((a) => a.span_id === s.span_id)
                .map((a, i) => (
                  <span
                    key={i}
                    className="ann"
                    style={{ left: `${pct(a.time_ns)}%`, background: levelColor(a.level) }}
                    title={`${elapsed(a.time_ns, originNs)} [${a.level}] ${a.source}: ${a.message}`}
                  />
                ))}
            </div>
            <div className="lane-outcome" data-outcome={s.outcome}>
              {s.outcome || '—'}
            </div>
          </div>
        );
      })}
    </div>
  );
}
