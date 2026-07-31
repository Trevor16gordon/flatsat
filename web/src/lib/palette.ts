/**
 * MATLAB's default line colour order.
 *
 * Chosen deliberately rather than for nostalgia: these seven are
 * distinguishable from one another, legible on both light and dark
 * plot backgrounds, and survive the common forms of colour blindness
 * better than a rainbow ramp does. Anyone who has read engineering
 * plots will also find them familiar, which is worth something.
 */
export const TRACE_COLORS = [
  '#0072BD',
  '#D95319',
  '#EDB120',
  '#7E2F8E',
  '#77AC30',
  '#4DBEEE',
  '#A2142F',
] as const;

export function traceColor(index: number): string {
  return TRACE_COLORS[index % TRACE_COLORS.length] as string;
}

/** Annotation levels map to intent, not to the trace palette. */
export function levelColor(level: string): string {
  if (level === 'error') return '#A2142F';
  if (level === 'warn') return '#D95319';
  return '#0072BD';
}

/** Span outcome tint for the timeline bars. */
export function outcomeColor(outcome: string): string {
  if (outcome === 'pass') return '#77AC30';
  if (outcome === 'fail') return '#A2142F';
  if (outcome === 'aborted') return '#D95319';
  return '#7f8c9a';
}
