/**
 * The source this app is being shaped for: an authenticated API over a
 * run database.
 *
 * NOT WIRED UP. It exists so the shape of the eventual server contract
 * is written down while the decisions are fresh, and so the seam is
 * exercised by having a second implementer of the interface rather than
 * one. Nothing constructs it yet.
 *
 * Notes for whoever finishes it:
 *  - Credentials belong in an httpOnly cookie set by the login flow, not
 *    in a token this module stores. A token in JS is a token in every
 *    XSS.
 *  - `loadRun` should return the run WITHOUT bulk series once the
 *    archive is large; the viewer already fetches channels lazily
 *    through `loadSeries`.
 *  - Commanding, if it ever lands, does NOT belong on this interface.
 *    Reading and acting want different authority, different audit and
 *    different review — see Capabilities in types.ts.
 */

import type { DataSource, TimeRange } from './source';
import type { Capabilities, MissionBlob, RunSummary, Series } from './types';

export class ApiDataSource implements DataSource {
  readonly label: string;
  private readonly baseUrl: string;

  constructor(baseUrl: string) {
    this.baseUrl = baseUrl.replace(/\/$/, '');
    this.label = `api: ${this.baseUrl}`;
  }

  capabilities(): Capabilities {
    // Browsing is the point of having a server; commanding stays off
    // until there is an authority model behind it worth trusting.
    return { canBrowseRuns: true, canCommand: false };
  }

  private async get<T>(path: string): Promise<T> {
    const res = await fetch(`${this.baseUrl}${path}`, { credentials: 'include' });
    if (!res.ok) throw new Error(`${res.status} ${res.statusText} for ${path}`);
    return (await res.json()) as T;
  }

  listRuns(): Promise<RunSummary[]> {
    return this.get<RunSummary[]>('/api/runs');
  }

  loadRun(runId: string): Promise<MissionBlob> {
    return this.get<MissionBlob>(`/api/runs/${encodeURIComponent(runId)}`);
  }

  loadSeries(
    runId: string,
    channel: string,
    range?: TimeRange,
    maxPoints?: number,
  ): Promise<Series | undefined> {
    const q = new URLSearchParams({ channel });
    if (range) {
      q.set('start_ns', String(range.startNs));
      q.set('end_ns', String(range.endNs));
    }
    if (maxPoints) q.set('max_points', String(maxPoints));
    return this.get<Series>(`/api/runs/${encodeURIComponent(runId)}/series?${q}`);
  }
}
