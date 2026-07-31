/**
 * The shape of a mission blob, mirroring `flatsat.telemetry.export`.
 *
 * These types are hand-mirrored from the Python exporter today. That is
 * a known duplication: the .proto files are the source of truth, and
 * generating this file from the descriptors is the obvious next step,
 * exactly as generating the exporter's TOPIC_TYPES map is. Until then,
 * a change to the blob format must be made in both places.
 */

/** How the data was produced. Sim, HIL and flight publish identical
 *  topics by design, so this is the only thing that tells them apart. */
export type SourceKind =
  | 'SOURCE_KIND_SIM'
  | 'SOURCE_KIND_HIL'
  | 'SOURCE_KIND_FLIGHT'
  | 'SOURCE_KIND_REPLAY'
  | 'SOURCE_KIND_UNSPECIFIED';

export interface RunIdentity {
  run_id: string;
  source_kind: SourceKind;
  mission_name: string;
  vehicle_path: string;
  vehicle_sha256: string;
  git_sha: string;
  /** Uncommitted changes were present: the run is not reproducible. */
  git_dirty: boolean;
  plant: string;
  host: string;
  started_wall_ns: number;
}

/** An interval. Nests via parent_span_id; `end_ns` null means the span
 *  never closed, which is the evidence that a run died inside it. */
export interface Span {
  span_id: string;
  parent_span_id: string;
  kind: string;
  name: string;
  start_ns: number | null;
  end_ns: number | null;
  outcome: string;
  detail: string;
  attributes: Record<string, string>;
}

/** A point event on the timeline. */
export interface Annotation {
  time_ns: number;
  span_id: string;
  level: string;
  source: string;
  message: string;
}

export interface TopicStats {
  count: number;
  first_ns: number;
  last_ns: number;
  rate_hz: number;
}

/** One plottable channel. `count` is the true sample count even when
 *  the points were decimated for transport. */
export interface Series {
  t_ns: number[];
  v: number[];
  count: number;
  decimated: boolean;
}

export interface MissionBlob {
  run: Partial<RunIdentity>;
  files: string[];
  spans: Span[];
  annotations: Annotation[];
  topics: Record<string, TopicStats>;
  series: Record<string, Series>;
}

/** Summary of a run, for a browse list. Today only ever one entry (the
 *  uploaded file); later, a page of rows from the archive database. */
export interface RunSummary {
  run_id: string;
  source_kind: SourceKind;
  mission_name: string;
  started_wall_ns: number;
}

/**
 * What the signed-in user is allowed to do.
 *
 * The viewer is read-only, and commanding a spacecraft from a browser
 * is not something to add by accident. Keeping it an explicit
 * capability means the eventual command path has one gate to find
 * rather than being spread across whichever component grew a button.
 */
export interface Capabilities {
  canBrowseRuns: boolean;
  canCommand: boolean;
}

export const READ_ONLY: Capabilities = { canBrowseRuns: false, canCommand: false };
