/**
 * The viewer shell.
 *
 * Everything reaches data through a DataSource, never through the
 * uploaded file directly — swapping to an authenticated API is meant to
 * be one constructor call in this file and nothing else.
 */

import { useCallback, useEffect, useMemo, useState } from 'react';
import { ChannelList } from './components/ChannelList';
import { PlotCanvas } from './components/PlotCanvas';
import { RunHeader } from './components/RunHeader';
import { Timeline } from './components/Timeline';
import type { DataSource } from './data/source';
import type { MissionBlob, Span } from './data/types';
import { parseBlob, UploadDataSource } from './data/uploadSource';
import { elapsed } from './lib/format';
import { levelColor, traceColor } from './lib/palette';

export default function App() {
  const [source, setSource] = useState<DataSource | null>(null);
  const [blob, setBlob] = useState<MissionBlob | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [selected, setSelected] = useState<string[]>([]);
  const [view, setView] = useState<[number, number] | null>(null);
  const [selectedSpan, setSelectedSpan] = useState<string | null>(null);
  const [dragOver, setDragOver] = useState(false);

  const ingest = useCallback(async (file: File) => {
    try {
      const parsed = parseBlob(await file.text());
      const src = new UploadDataSource(parsed, file.name);
      setSource(src);
      setBlob(await src.loadRun(''));
      setError(null);
      setSelected([]);
      setView(null);
      setSelectedSpan(null);
    } catch (err) {
      setError((err as Error).message);
      setBlob(null);
    }
  }, []);

  // Full extent of the run, used when no zoom is applied.
  const extent = useMemo((): [number, number] | null => {
    if (!blob) return null;
    let lo = Infinity;
    let hi = -Infinity;
    for (const s of Object.values(blob.series)) {
      if (s.t_ns.length === 0) continue;
      lo = Math.min(lo, s.t_ns[0] as number);
      hi = Math.max(hi, s.t_ns[s.t_ns.length - 1] as number);
    }
    for (const s of blob.spans) {
      if (s.start_ns !== null) lo = Math.min(lo, s.start_ns);
      if (s.end_ns !== null) hi = Math.max(hi, s.end_ns);
    }
    return Number.isFinite(lo) ? [lo, hi] : null;
  }, [blob]);

  const originNs = extent ? extent[0] : 0;
  const window_: [number, number] = view ?? extent ?? [0, 1];

  const channels = useMemo(() => {
    if (!blob) return [];
    return selected
      .map((key, i) => {
        const series = blob.series[key];
        return series ? { key, series, color: traceColor(i) } : null;
      })
      .filter((c): c is NonNullable<typeof c> => c !== null);
  }, [blob, selected]);

  const marks = useMemo(() => {
    if (!blob) return [];
    return blob.annotations.map((a) => ({
      timeNs: a.time_ns,
      color: levelColor(a.level),
      label: a.message,
    }));
  }, [blob]);

  const toggle = (key: string) =>
    setSelected((prev) => (prev.includes(key) ? prev.filter((k) => k !== key) : [...prev, key]));

  // Clicking a span zooms the plot to it — the reason the timeline and
  // the plot share one axis.
  const focusSpan = (span: Span) => {
    setSelectedSpan(span.span_id);
    if (span.start_ns !== null) {
      setView([span.start_ns, span.end_ns ?? window_[1]]);
    }
  };

  useEffect(() => {
    const stop = (e: DragEvent) => e.preventDefault();
    window.addEventListener('dragover', stop);
    window.addEventListener('drop', stop);
    return () => {
      window.removeEventListener('dragover', stop);
      window.removeEventListener('drop', stop);
    };
  }, []);

  return (
    <div
      className={`app${dragOver ? ' dragover' : ''}`}
      onDragOver={(e) => {
        e.preventDefault();
        setDragOver(true);
      }}
      onDragLeave={() => setDragOver(false)}
      onDrop={(e) => {
        e.preventDefault();
        setDragOver(false);
        const file = e.dataTransfer.files[0];
        if (file) void ingest(file);
      }}
    >
      <header className="topbar">
        <div className="brand">
          flatsat<span className="brand-sub">mission viewer</span>
        </div>
        <label className="load-btn">
          load run…
          <input
            type="file"
            accept="application/json,.json"
            onChange={(e) => {
              const file = e.target.files?.[0];
              if (file) void ingest(file);
            }}
          />
        </label>
        <div className="source-label">{source?.label ?? 'no source'}</div>
        <div className="readonly" title="commanding is not enabled in this build">
          read-only
        </div>
      </header>

      {error && <div className="error-bar">could not read that file — {error}</div>}

      {!blob && !error && (
        <div className="dropzone">
          <div className="dropzone-inner">
            <p className="dz-title">drop a mission blob here</p>
            <p className="dz-sub">
              produced by <code>python -m flatsat.telemetry.export &lt;run-dir&gt;</code>
            </p>
          </div>
        </div>
      )}

      {blob && (
        <>
          <RunHeader run={blob.run} fileCount={blob.files.length} />
          <div className="body">
            <aside className="sidebar">
              <ChannelList blob={blob} selected={selected} onToggle={toggle} />
            </aside>
            <main className="main">
              <section className="panel">
                <div className="panel-head">
                  <span>timeline</span>
                  <span className="panel-note">
                    {elapsed(window_[0], originNs)} → {elapsed(window_[1], originNs)}
                    {view && (
                      <button className="link" onClick={() => setView(null)}>
                        reset zoom
                      </button>
                    )}
                  </span>
                </div>
                <Timeline
                  spans={blob.spans}
                  annotations={blob.annotations}
                  originNs={originNs}
                  window={window_}
                  onSelectSpan={focusSpan}
                  selectedSpanId={selectedSpan}
                />
              </section>

              <section className="panel grow">
                <div className="panel-head">
                  <span>signals</span>
                  <span className="panel-note">
                    {channels.length} of {Object.keys(blob.series).length} channels
                  </span>
                </div>
                <PlotCanvas
                  channels={channels}
                  originNs={originNs}
                  viewNs={view}
                  onViewChange={setView}
                  marks={marks}
                />
              </section>

              <section className="panel">
                <div className="panel-head">
                  <span>events</span>
                  <span className="panel-note">{blob.annotations.length}</span>
                </div>
                <div className="events">
                  {blob.annotations.length === 0 && (
                    <div className="empty-note">no annotations in this run</div>
                  )}
                  {blob.annotations.map((a, i) => (
                    <div key={i} className={`event level-${a.level}`}>
                      <span className="event-time">{elapsed(a.time_ns, originNs)}</span>
                      <span className="event-source">{a.source}</span>
                      <span className="event-msg">{a.message}</span>
                    </div>
                  ))}
                </div>
              </section>
            </main>
          </div>
        </>
      )}
    </div>
  );
}
