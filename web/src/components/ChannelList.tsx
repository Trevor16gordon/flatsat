/**
 * Channel picker. A run has tens of channels and a real vehicle will
 * have hundreds, so this is a filter box first and a list second.
 */

import { useMemo, useState } from 'react';
import type { MissionBlob } from '../data/types';

interface Props {
  blob: MissionBlob;
  selected: string[];
  onToggle: (key: string) => void;
}

export function ChannelList({ blob, selected, onToggle }: Props) {
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
                <label key={key} className="channel-row">
                  <input
                    type="checkbox"
                    checked={selected.includes(key)}
                    onChange={() => onToggle(key)}
                  />
                  <span className="channel-name">{key.slice(topic.length + 1)}</span>
                  <span className="channel-count" title={`${s?.count ?? 0} samples`}>
                    {s?.decimated ? '~' : ''}
                    {s?.count ?? 0}
                  </span>
                </label>
              );
            })}
          </div>
        ))}
        {groups.length === 0 && <div className="empty-note">no channels match</div>}
      </div>
    </div>
  );
}
