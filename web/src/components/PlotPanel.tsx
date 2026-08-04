/**
 * One plot in the grid: legend, axis-link toggles, and the drop target.
 *
 * The drop zones are the docking gesture: a channel released over the
 * body joins this plot; released near an edge it docks a NEW plot on
 * that side. The zone under the pointer is highlighted while dragging,
 * so the gesture is legible before it is committed.
 */

import { useMemo, useState } from 'react';
import type { DragEvent } from 'react';
import type { PlotSpec } from '../App';
import type { MissionBlob } from '../data/types';
import { shortChannel } from '../lib/format';
import { traceColor } from '../lib/palette';
import { PlotCanvas } from './PlotCanvas';

export type DropZone = 'center' | 'top' | 'bottom' | 'left' | 'right';

const CHANNEL_MIME = 'application/x-flatsat-channel';
/** Fraction of the panel's size that counts as an edge for docking. */
const EDGE = 0.18;

interface Props {
  plot: PlotSpec;
  blob: MissionBlob;
  originNs: number;
  extent: [number, number] | null;
  sharedX: [number, number] | null;
  sharedY: [number, number] | null;
  focused: boolean;
  marks: { timeNs: number; color: string; label: string }[];
  onFocus: () => void;
  onPatch: (patch: Partial<PlotSpec>) => void;
  onSharedX: (view: [number, number] | null) => void;
  onSharedY: (view: [number, number] | null) => void;
  onDropChannel: (zone: DropZone, key: string) => void;
  onRemoveChannel: (key: string) => void;
}

export function PlotPanel({
  plot,
  blob,
  originNs,
  extent,
  sharedX,
  sharedY,
  focused,
  marks,
  onFocus,
  onPatch,
  onSharedX,
  onSharedY,
  onDropChannel,
  onRemoveChannel,
}: Props) {
  const [zone, setZone] = useState<DropZone | null>(null);

  const channels = useMemo(
    () =>
      plot.channels
        .map((key, i) => {
          const series = blob.series[key];
          return series ? { key, series, color: traceColor(i) } : null;
        })
        .filter((c): c is NonNullable<typeof c> => c !== null),
    [blob, plot.channels],
  );

  const viewX = plot.xLink ? sharedX : plot.xView;
  const viewY = plot.yLink ? sharedY : plot.yView;

  const setViewX = (view: [number, number] | null) =>
    plot.xLink ? onSharedX(view) : onPatch({ xView: view });
  const setViewY = (view: [number, number] | null) =>
    plot.yLink ? onSharedY(view) : onPatch({ yView: view });

  const zoneAt = (e: DragEvent): DropZone => {
    const rect = e.currentTarget.getBoundingClientRect();
    const fx = (e.clientX - rect.left) / rect.width;
    const fy = (e.clientY - rect.top) / rect.height;
    // Nearest edge wins, but only inside its band; ties favor the
    // vertical split because rows are the coarser structure.
    if (fy < EDGE) return 'top';
    if (fy > 1 - EDGE) return 'bottom';
    if (fx < EDGE) return 'left';
    if (fx > 1 - EDGE) return 'right';
    return 'center';
  };

  return (
    <div
      className={`plot-panel${focused ? ' focused' : ''}${zone ? ` drop-${zone}` : ''}`}
      onMouseDown={onFocus}
      onDragOver={(e) => {
        if (!e.dataTransfer.types.includes(CHANNEL_MIME)) return;
        e.preventDefault();
        e.stopPropagation();
        setZone(zoneAt(e));
      }}
      onDragLeave={() => setZone(null)}
      onDrop={(e) => {
        const key = e.dataTransfer.getData(CHANNEL_MIME);
        setZone(null);
        if (!key) return;
        e.preventDefault();
        e.stopPropagation();
        onDropChannel(zoneAt(e), key);
      }}
    >
      <div className="plot-panel-head">
        <div className="legend">
          {channels.map((ch) => (
            <span key={ch.key} className="legend-chip" title={ch.key}>
              <span className="swatch" style={{ background: ch.color }} />
              {shortChannel(ch.key)}
              <button
                className="chip-x"
                title="remove channel"
                onClick={() => onRemoveChannel(ch.key)}
              >
                ×
              </button>
            </span>
          ))}
        </div>
        <div className="axis-links">
          <button
            className={`link-btn${plot.xLink ? ' on' : ''}`}
            title="link time axis to the other plots (shared pan/zoom)"
            onClick={() => onPatch({ xLink: !plot.xLink, xView: plot.xLink ? sharedX : null })}
          >
            link t
          </button>
          <button
            className={`link-btn${plot.yLink ? ' on' : ''}`}
            title="link value axis to the other plots (shared pan/zoom)"
            onClick={() => onPatch({ yLink: !plot.yLink, yView: plot.yLink ? sharedY : null })}
          >
            link y
          </button>
        </div>
      </div>
      <PlotCanvas
        channels={channels}
        originNs={originNs}
        viewNs={viewX ?? extent}
        viewV={viewY}
        onViewChange={setViewX}
        onViewVChange={setViewY}
        marks={marks}
      />
      {zone && <div className={`drop-hint drop-hint-${zone}`} />}
    </div>
  );
}
