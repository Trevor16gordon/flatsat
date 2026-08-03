/**
 * A run the user handed us directly — today's only source.
 *
 * The blob arrives whole, so windowing and decimation hints are ignored
 * here: the exporter already decimated on the way out. An API source
 * will honour them.
 */

import type { DataSource } from './source';
import type { Capabilities, MissionBlob, RunSummary, Series } from './types';
import { READ_ONLY } from './types';

/** Reject anything that does not look like an exported run, rather than
 *  rendering a blank page and leaving the user to guess why. */
export function parseBlob(text: string): MissionBlob {
  let raw: unknown;
  try {
    raw = JSON.parse(text);
  } catch (err) {
    throw new Error(`not valid JSON: ${(err as Error).message}`);
  }
  const blob = raw as Partial<MissionBlob>;
  if (!blob || typeof blob !== 'object' || !blob.series || !blob.topics) {
    throw new Error(
      'missing "series" and "topics" — is this the output of `python -m flatsat.telemetry.export`?',
    );
  }
  return {
    run: blob.run ?? {},
    files: blob.files ?? [],
    spans: blob.spans ?? [],
    annotations: blob.annotations ?? [],
    topics: blob.topics,
    series: blob.series,
  };
}

export class UploadDataSource implements DataSource {
  readonly label: string;
  private readonly blob: MissionBlob;

  constructor(blob: MissionBlob, filename: string) {
    this.blob = blob;
    this.label = `file: ${filename}`;
  }

  capabilities(): Capabilities {
    return READ_ONLY;
  }

  async listRuns(): Promise<RunSummary[]> {
    const run = this.blob.run;
    return [
      {
        run_id: run.run_id ?? '(unnamed run)',
        source_kind: run.source_kind ?? 'SOURCE_KIND_UNSPECIFIED',
        mission_name: run.mission_name ?? '',
        started_wall_ns: run.started_wall_ns ?? 0,
      },
    ];
  }

  async loadRun(_runId: string): Promise<MissionBlob> {
    return this.blob;
  }

  async loadSeries(_runId: string, channel: string): Promise<Series | undefined> {
    return this.blob.series[channel];
  }
}
