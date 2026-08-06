/**
 * The viewer shell.
 *
 * Everything reaches data through a DataSource, never through the
 * uploaded file directly — swapping to an authenticated API is meant to
 * be one constructor call in this file and nothing else.
 *
 * Plots are a grid of rows. Channels are DRAGGED in: onto a plot's body
 * to add a trace, onto a plot's edge to dock a new plot on that side
 * (top/bottom insert a row, left/right split the row). Each plot can
 * link or unlink its time and value axes from the shared views, so
 * panning one linked plot pans them all.
 */

import { useCallback, useEffect, useMemo, useState } from 'react';
import { ChannelList } from './components/ChannelList';
import { CommandPanel } from './components/CommandPanel';
import { OrbitPanel } from './components/OrbitPanel';
import { PlotPanel } from './components/PlotPanel';
import type { DropZone } from './components/PlotPanel';
import { RunHeader } from './components/RunHeader';
import { Timeline } from './components/Timeline';
import type { DataSource } from './data/source';
import type { MissionBlob, Span } from './data/types';
import { ApiDataSource } from './data/apiSource';
import { parseBlob, UploadDataSource } from './data/uploadSource';
import { elapsed } from './lib/format';
import { levelColor } from './lib/palette';

export interface PlotSpec {
  id: string;
  channels: string[];
  /** Follow the shared time window (pan/zoom linked across plots). */
  xLink: boolean;
  /** Follow the shared value window. */
  yLink: boolean;
  /** Private windows, used when the matching link is off. */
  xView: [number, number] | null;
  yView: [number, number] | null;
}

let plotCounter = 0;
const newPlot = (channels: string[]): PlotSpec => ({
  id: `plot-${plotCounter++}`,
  channels,
  xLink: true,
  yLink: false,
  xView: null,
  yView: null,
});

export default function App() {
  const [source, setSource] = useState<DataSource | null>(null);
  const [blob, setBlob] = useState<MissionBlob | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [plots, setPlots] = useState<PlotSpec[]>([]);
  const [rows, setRows] = useState<string[][]>([]);
  const [focusedPlot, setFocusedPlot] = useState<string | null>(null);
  const [sharedX, setSharedX] = useState<[number, number] | null>(null);
  const [sharedY, setSharedY] = useState<[number, number] | null>(null);
  const [selectedSpan, setSelectedSpan] = useState<string | null>(null);
  const [dragOver, setDragOver] = useState(false);
  const [live, setLive] = useState(false);
  const [apiBase, setApiBase] = useState<string | null>(null);
  const [canCommand, setCanCommand] = useState(false);
  const [nowMs, setNowMs] = useState(() => Date.now());

  // Live mode: ?api=http://localhost:8600 points the viewer at a
  // ground-bridge server and polls it — the blob grows as passes land.
  // Plot layout survives refreshes because it references channels by
  // name, never by blob identity.
  useEffect(() => {
    const api = new URLSearchParams(window.location.search).get('api');
    if (!api) return;
    const src = new ApiDataSource(api);
    setSource(src);
    const base = api.replace(/\/$/, '');
    setApiBase(base);
    let cancelled = false;
    let capsKnown = false;
    // Capabilities retry with every poll until they succeed — a page
    // loaded before the bridge must not hide the commanding panel
    // forever over one failed fetch.
    const fetchCaps = async () => {
      if (capsKnown) return;
      try {
        const r = await fetch(`${base}/api/capabilities`, { credentials: 'include' });
        const caps = (await r.json()) as { canCommand?: boolean };
        if (!cancelled) {
          setCanCommand(Boolean(caps.canCommand));
          capsKnown = true;
        }
      } catch {
        /* bridge not up yet — retry on the next poll */
      }
    };
    const poll = async () => {
      void fetchCaps();
      try {
        const runs = await src.listRuns();
        const first = runs[0];
        if (!first || cancelled) return;
        const fresh = await src.loadRun(first.run_id);
        if (!cancelled) {
          setBlob(fresh);
          setError(null);
          setLive(true);
        }
      } catch {
        // Transient poll failures stay quiet; a dead bridge just means
        // the view stops growing, exactly like a quiet link.
      }
    };
    void poll();
    const timer = setInterval(() => void poll(), 3000);
    const tick = setInterval(() => setNowMs(Date.now()), 1000);
    return () => {
      cancelled = true;
      clearInterval(timer);
      clearInterval(tick);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const ingest = useCallback(async (file: File) => {
    try {
      const parsed = parseBlob(await file.text());
      const src = new UploadDataSource(parsed, file.name);
      setSource(src);
      setBlob(await src.loadRun(''));
      setError(null);
      setPlots([]);
      setRows([]);
      setFocusedPlot(null);
      setSharedX(null);
      setSharedY(null);
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
  const window_: [number, number] = sharedX ?? extent ?? [0, 1];

  const marks = useMemo(() => {
    if (!blob) return [];
    return blob.annotations.map((a) => ({
      timeNs: a.time_ns,
      color: levelColor(a.level),
      label: a.message,
    }));
  }, [blob]);

  const plottedKeys = useMemo(() => new Set(plots.flatMap((p) => p.channels)), [plots]);

  const updatePlot = (id: string, patch: Partial<PlotSpec>) =>
    setPlots((prev) => prev.map((p) => (p.id === id ? { ...p, ...patch } : p)));

  /** Drop `plots` entries no row references and rows with no plots. */
  const collapse = (nextRows: string[][], nextPlots: PlotSpec[]) => {
    const live = new Set(nextRows.flat());
    setRows(nextRows.filter((r) => r.length > 0));
    setPlots(nextPlots.filter((p) => live.has(p.id)));
  };

  const addChannel = (plotId: string, key: string) => {
    setPlots((prev) =>
      prev.map((p) =>
        p.id === plotId && !p.channels.includes(key)
          ? { ...p, channels: [...p.channels, key] }
          : p,
      ),
    );
    setFocusedPlot(plotId);
  };

  const removeChannel = (plotId: string, key: string) => {
    const plot = plots.find((p) => p.id === plotId);
    if (!plot) return;
    const remaining = plot.channels.filter((k) => k !== key);
    if (remaining.length > 0) {
      updatePlot(plotId, { channels: remaining });
      return;
    }
    collapse(
      rows.map((r) => r.filter((id) => id !== plotId)),
      plots,
    );
  };

  /** A channel dropped on a plot: its body adds, its edges dock a new plot. */
  const dropChannel = (plotId: string, zone: DropZone, key: string) => {
    if (zone === 'center') {
      addChannel(plotId, key);
      return;
    }
    const plot = newPlot([key]);
    const rowIdx = rows.findIndex((r) => r.includes(plotId));
    if (rowIdx < 0) return;
    const nextRows = rows.map((r) => [...r]);
    if (zone === 'top' || zone === 'bottom') {
      nextRows.splice(rowIdx + (zone === 'bottom' ? 1 : 0), 0, [plot.id]);
    } else {
      const row = nextRows[rowIdx];
      if (!row) return;
      const col = row.indexOf(plotId);
      row.splice(col + (zone === 'right' ? 1 : 0), 0, plot.id);
    }
    setRows(nextRows);
    setPlots((prev) => [...prev, plot]);
    setFocusedPlot(plot.id);
  };

  /** A channel landing in the empty signals area starts the first plot. */
  const dropIntoEmpty = (key: string) => {
    const plot = newPlot([key]);
    setRows((prev) => [...prev, [plot.id]]);
    setPlots((prev) => [...prev, plot]);
    setFocusedPlot(plot.id);
  };

  /** Clicking a channel adds it to the focused plot (drag is the main path). */
  const clickChannel = (key: string) => {
    const target = plots.find((p) => p.id === focusedPlot) ?? plots[plots.length - 1];
    if (!target) {
      dropIntoEmpty(key);
      return;
    }
    if (target.channels.includes(key)) {
      removeChannel(target.id, key);
      return;
    }
    addChannel(target.id, key);
  };

  // Clicking a span zooms the shared time axis to it — the reason the
  // timeline and the linked plots share one axis.
  const focusSpan = (span: Span) => {
    setSelectedSpan(span.span_id);
    if (span.start_ns !== null) {
      setSharedX([span.start_ns, span.end_ns ?? window_[1]]);
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
        // Only a FILE drag restyles the whole app; channel drags are
        // handled (and highlighted) by the plots themselves.
        if (e.dataTransfer.types.includes('Files')) setDragOver(true);
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
        {live && blob && (() => {
          // Freshest downlinked sample = the last thing that CROSSED the
          // link. Age is honest liveness: it grows between passes and
          // snaps back when one lands.
          let lastNs = 0;
          for (const s of Object.values(blob.topics)) {
            if (s.last_ns > lastNs) lastNs = s.last_ns;
          }
          const ageS = Math.max(0, (nowMs - lastNs / 1e6) / 1000);
          const state = ageS < 15 ? 'fresh' : ageS < 45 ? 'waiting' : 'stale';
          return (
            <div className={`live-badge live-${state}`} title="age of the newest downlinked sample">
              <span className="live-dot" />
              LIVE · downlink {ageS < 1 ? 'now' : `${Math.floor(ageS)}s ago`}
            </div>
          );
        })()}
        <div className="readonly" title="commanding is not enabled in this build">
          read-only
        </div>
      </header>

      {error && <div className="error-bar">could not read that file — {error}</div>}

      {!blob && (
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
              <ChannelList blob={blob} plotted={plottedKeys} onPick={clickChannel} />
            </aside>
            <main className="main">
              <section className="panel">
                <div className="panel-head">
                  <span>timeline</span>
                  <span className="panel-note">
                    {elapsed(window_[0], originNs)} → {elapsed(window_[1], originNs)}
                    {sharedX && (
                      <button className="link" onClick={() => setSharedX(null)}>
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

              <OrbitPanel blob={blob} originNs={originNs} window={window_} />

              <section className="panel grow">
                <div className="panel-head">
                  <span>signals</span>
                  <span className="panel-note">
                    {plottedKeys.size} of {Object.keys(blob.series).length} channels ·{' '}
                    {plots.length} plot{plots.length === 1 ? '' : 's'}
                  </span>
                </div>
                {rows.length === 0 ? (
                  <div
                    className="plot-empty-stage"
                    onDragOver={(e) => e.preventDefault()}
                    onDrop={(e) => {
                      e.preventDefault();
                      const key = e.dataTransfer.getData('application/x-flatsat-channel');
                      if (key) dropIntoEmpty(key);
                    }}
                  >
                    <span>drag a channel here to start a plot</span>
                  </div>
                ) : (
                  <div className="plot-grid">
                    {rows.map((row, i) => (
                      <div key={i} className="plot-row">
                        {row.map((id) => {
                          const plot = plots.find((p) => p.id === id);
                          if (!plot) return null;
                          return (
                            <PlotPanel
                              key={id}
                              plot={plot}
                              blob={blob}
                              originNs={originNs}
                              extent={extent}
                              sharedX={sharedX}
                              sharedY={sharedY}
                              focused={focusedPlot === id}
                              marks={marks}
                              onFocus={() => setFocusedPlot(id)}
                              onPatch={(patch) => updatePlot(id, patch)}
                              onSharedX={setSharedX}
                              onSharedY={setSharedY}
                              onDropChannel={(zone, key) => dropChannel(id, zone, key)}
                              onRemoveChannel={(key) => removeChannel(id, key)}
                            />
                          );
                        })}
                      </div>
                    ))}
                  </div>
                )}
              </section>

              {canCommand && apiBase && <CommandPanel apiBase={apiBase} blob={blob} />}

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
