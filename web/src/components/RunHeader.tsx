/**
 * Provenance banner.
 *
 * Source kind and git dirtiness are given real visual weight on
 * purpose: a SIM run mistaken for a FLIGHT run, or a plot from
 * uncommitted code treated as reproducible, are the two ways this
 * viewer could actively mislead someone.
 */

import type { RunIdentity } from '../data/types';
import { wallClock } from '../lib/format';

const KIND_LABEL: Record<string, string> = {
  SOURCE_KIND_SIM: 'SIM',
  SOURCE_KIND_HIL: 'HIL',
  SOURCE_KIND_FLIGHT: 'FLIGHT',
  SOURCE_KIND_REPLAY: 'REPLAY',
  SOURCE_KIND_UNSPECIFIED: 'UNKNOWN',
};

export function RunHeader({ run, fileCount }: { run: Partial<RunIdentity>; fileCount: number }) {
  const kind = run.source_kind ?? 'SOURCE_KIND_UNSPECIFIED';
  return (
    <div className="runhead">
      <span className={`kind kind-${KIND_LABEL[kind]?.toLowerCase() ?? 'unknown'}`}>
        {KIND_LABEL[kind] ?? 'UNKNOWN'}
      </span>
      <span className="run-id" title={run.run_id}>
        {run.run_id ?? '(no run id)'}
      </span>
      <dl className="facts">
        <dt>mission</dt>
        <dd>{run.mission_name || '—'}</dd>
        <dt>vehicle</dt>
        <dd title={run.vehicle_sha256}>{run.vehicle_path || '—'}</dd>
        <dt>plant</dt>
        <dd>{run.plant || '—'}</dd>
        <dt>host</dt>
        <dd>{run.host || '—'}</dd>
        <dt>started</dt>
        <dd>{wallClock(run.started_wall_ns ?? 0)}</dd>
        <dt>code</dt>
        <dd>
          {run.git_sha || '(unknown)'}
          {run.git_dirty && <span className="dirty" title="uncommitted changes: not reproducible">dirty</span>}
        </dd>
        <dt>files</dt>
        <dd>{fileCount}</dd>
      </dl>
    </div>
  );
}
