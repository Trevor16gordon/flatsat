/**
 * Mission-control commanding, browser edition.
 *
 * Every button here POSTs to the ground bridge, which publishes into
 * the ground namespace — commands queue at the ground station and
 * cross the space link at the next contact window, exactly like the
 * CLI tools. Nothing in this panel talks to the spacecraft directly;
 * the wait for the pass IS the product being demonstrated.
 *
 * The console section is the exception and says so: clearing the view
 * is ground-side housekeeping, not a spacecraft action.
 */

import { useCallback, useState } from 'react';
import type { MissionBlob } from '../data/types';

const MODE_NAMES: Record<number, string> = {
  0: 'UNSPECIFIED',
  1: 'INIT',
  2: 'NOMINAL',
  3: 'SAFE',
  4: 'RECOVERY',
};

interface Props {
  apiBase: string;
  blob: MissionBlob;
}

/** The last value of a channel, or undefined when absent. */
function lastValue(blob: MissionBlob, channel: string): number | undefined {
  const s = blob.series[channel];
  return s && s.v.length > 0 ? s.v[s.v.length - 1] : undefined;
}

export function CommandPanel({ apiBase, blob }: Props) {
  const [busy, setBusy] = useState(false);
  const [result, setResult] = useState<string | null>(null);
  const [reason, setReason] = useState('');
  const [uploadVersion, setUploadVersion] = useState('');
  const [activateTarget, setActivateTarget] = useState('');

  const post = useCallback(
    async (path: string, body: object) => {
      setBusy(true);
      try {
        const res = await fetch(`${apiBase}${path}`, {
          method: 'POST',
          credentials: 'include',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        });
        const parsed = (await res.json()) as { ok: boolean; detail: string };
        setResult(`${parsed.ok ? '✓' : '✗'} ${parsed.detail}`);
      } catch (e) {
        setResult(`✗ ground station unreachable — ${String(e)}`);
      } finally {
        setBusy(false);
      }
    },
    [apiBase],
  );

  const requestMode = (mode: string) => {
    void post('/api/command/mode', {
      mode,
      reason: reason || 'mission control',
      ground: true,
    });
  };

  const activate = () => {
    if (!activateTarget) return;
    const [name, version] = activateTarget.split('@');
    const warning =
      `Activate ${activateTarget}?\n\n` +
      'This puts the artifact in control authority after the flight side ' +
      'adopts it (component restart).';
    if (!window.confirm(warning)) return;
    void post('/api/command/artifact', { action: 'activate', name, version, ground: true });
  };

  const rollback = () => {
    const name = activateTarget ? activateTarget.split('@')[0] : 'ml_detumble';
    void post('/api/command/artifact', { action: 'rollback', name, ground: true });
  };

  const upload = (file: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      const b64 = String(reader.result).split(',')[1] ?? '';
      const version = uploadVersion || new Date().toISOString().slice(0, 10);
      void post('/api/command/upload', {
        name: 'ml_detumble',
        version,
        kind: 'model',
        content_b64: b64,
      });
    };
    reader.readAsDataURL(file);
  };

  const modeValue = lastValue(blob, 'downlink/mode.mode');
  const modeName = modeValue !== undefined ? (MODE_NAMES[modeValue] ?? '?') : 'unknown';
  const staged = blob.annotations
    .filter((a) => a.message.startsWith('staged: '))
    .at(-1)
    ?.message.replace('staged: ', '')
    .split(', ')
    .filter((s) => s && s !== 'nothing');

  return (
    <section className="command-panel">
      <div className="cmd-header">
        COMMANDING
        <span className="cmd-note">every command crosses the link at the next pass</span>
      </div>
      <div className="cmd-row">
        <span className="cmd-label">
          MODE <b className={`cmd-mode cmd-mode-${modeName.toLowerCase()}`}>{modeName}</b>
        </span>
        <input
          className="cmd-input"
          placeholder="reason (recorded in the transition)"
          value={reason}
          onChange={(e) => setReason(e.target.value)}
        />
        {['RECOVERY', 'NOMINAL', 'SAFE'].map((m) => (
          <button key={m} className="cmd-btn" disabled={busy} onClick={() => requestMode(m)}>
            → {m}
          </button>
        ))}
      </div>
      <div className="cmd-row">
        <span className="cmd-label">DEPLOY</span>
        <select
          className="cmd-input"
          value={activateTarget}
          onChange={(e) => setActivateTarget(e.target.value)}
        >
          <option value="">staged artifact…</option>
          {(staged ?? []).map((s) => (
            <option key={s} value={s}>
              {s}
            </option>
          ))}
        </select>
        <button className="cmd-btn" disabled={busy || !activateTarget} onClick={activate}>
          activate
        </button>
        <button className="cmd-btn" disabled={busy} onClick={rollback}>
          rollback
        </button>
        <input
          className="cmd-input"
          placeholder="upload version tag"
          value={uploadVersion}
          onChange={(e) => setUploadVersion(e.target.value)}
        />
        <label className="cmd-btn">
          upload…
          <input
            type="file"
            style={{ display: 'none' }}
            onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) upload(f);
              e.target.value = '';
            }}
          />
        </label>
      </div>
      <div className="cmd-row cmd-console">
        <span className="cmd-label">CONSOLE</span>
        <button
          className="cmd-btn"
          disabled={busy}
          onClick={() => void post('/api/console/clear', {})}
        >
          clear view
        </button>
        <span className="cmd-note">ground-side housekeeping only — nothing reaches the vehicle</span>
      </div>
      {result && <div className="cmd-result">{result}</div>}
    </section>
  );
}
