/**
 * Channel picker. A run has tens of channels and a real vehicle will
 * have hundreds, so this is a filter box first and a list second.
 *
 * Rows are DRAG SOURCES: drop one on a plot to add the trace, on a
 * plot's edge to dock a new plot there, or on the empty stage to start
 * the first one. Clicking still works as a shortcut — it toggles the
 * channel on the focused plot.
 */

import { useMemo, useState } from 'react';
import type { MissionBlob } from '../data/types';

const CHANNEL_MIME = 'application/x-flatsat-channel';

interface Props {
  blob: MissionBlob;
  /** Channels currently on any plot, for the plotted marker. */
  plotted: Set<string>;
  /** Click fallback: toggle on the focused plot. */
  onPick: (key: string) => void;
}

export function ChannelList({ blob, plotted, onPick }: Props) {
  const [query, setQuery] = useState('');

  const groups = useMemo(() => {
    const byTopic = new Map<string, string[]>();
    for (const key of Object.keys(blob.series).sort()) {
      if (query && !key.toLowerCase().includes(query.toLowerCase())) continue;
      const dot = key.lastIndexOf('.');
      const topic = dot < 0 ? key : key.slice(0, dot);
      const list = byTopic.get(topic) ?? [];
      list.push(key);
      byTopic.set(topic, list);
    }
    return [...byTopic.entries()];
  }, [blob.series, query]);

  return (
    <div className="channels">
      <input
        className="filter"
        placeholder="filter channels…"
        value={query}
        onChange={(e) => setQuery(e.target.value)}
      />
      <div className="channel-scroll">
        {groups.map(([topic, keys]) => (
          <div key={topic} className="channel-group">
            <div className="channel-topic" title={topic}>
              {topic}
              <span className="rate">{blob.topics[topic]?.rate_hz ?? '—'} Hz</span>
            </div>
            {keys.map((key) => {
              const s = blob.series[key];
              return (
                <div
                  key={key}
                  className={`channel-row${plotted.has(key) ? ' plotted' : ''}`}
                  draggable
                  onDragStart={(e) => {
                    e.dataTransfer.setData(CHANNEL_MIME, key);
                    e.dataTransfer.effectAllowed = 'copy';
                  }}
                  onClick={() => onPick(key)}
                  title="drag onto a plot, or click for the focused plot"
                >
                  <span className="plotted-dot" />
                  <span className="channel-name">{key.slice(topic.length + 1)}</span>
                  <span className="channel-count" title={`${s?.count ?? 0} samples`}>
                    {s?.decimated ? '~' : ''}
                    {s?.count ?? 0}
                  </span>
                </div>
              );
            })}
          </div>
        ))}
        {groups.length === 0 && <div className="empty-note">no channels match</div>}
      </div>
    </div>
  );
}
