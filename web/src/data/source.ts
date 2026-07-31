/**
 * Where runs come from — the seam the whole app is built around.
 *
 * Today the only source is a file the user uploads. Tomorrow it is a
 * database behind an authenticated API, and later still it may be a
 * live session streaming from a spacecraft. Every one of those is the
 * same three questions: what runs exist, give me one, and give me a
 * channel over a time range.
 *
 * The time range on `loadSeries` is not currently used by the upload
 * source — the blob arrives whole and pre-decimated. It is in the
 * interface anyway, because server-side decimation over a requested
 * window is the entire reason an API source would exist, and retrofitting
 * a parameter through a UI is much worse than ignoring one.
 */

import type { Capabilities, MissionBlob, RunSummary, Series } from './types';

export interface TimeRange {
  /** Absolute nanoseconds, inclusive. */
  startNs: number;
  endNs: number;
}

export interface DataSource {
  /** Human-readable name, shown in the status bar. */
  readonly label: string;

  /** What this source lets the user do. */
  capabilities(): Capabilities;

  /** Runs available to browse. Upload sources return what is loaded. */
  listRuns(): Promise<RunSummary[]>;

  /** Fetch one run's blob, without bulk series if the source can defer them. */
  loadRun(runId: string): Promise<MissionBlob>;

  /**
   * Fetch one channel, optionally windowed and decimated server-side.
   *
   * @param runId - Which run.
   * @param channel - Dotted channel key, e.g. `hal/imu0/sample.gyro_x_rad_s`.
   * @param range - Window of interest; omitted means the whole run.
   * @param maxPoints - Hint for how much resolution the caller can draw.
   */
  loadSeries(
    runId: string,
    channel: string,
    range?: TimeRange,
    maxPoints?: number,
  ): Promise<Series | undefined>;
}
