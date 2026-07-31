# flatsat mission viewer

A React app for reading mission blobs produced by
`python -m flatsat.telemetry.export`. Self-contained: its own
`package.json`, TypeScript config and lint rules, so the Python repo's
pre-commit gates never touch it and this can be deployed on its own.

```bash
npm install
npm run dev      # http://localhost:5173
npm run build    # -> dist/, static files for any host
```

Then drag a `mission.json` onto the window, or use **load run…**.

## What it shows

- **Provenance banner** — run id, source kind (SIM / HIL / FLIGHT), vehicle
  path and digest as flown, git sha and a `dirty` badge. Source kind and
  dirtiness are given visual weight deliberately: a SIM run mistaken for a
  FLIGHT one, or a plot from uncommitted code treated as reproducible, are
  the two ways this app could actively mislead.
- **Timeline** — mission and sub-mission spans as nested swimlanes, with
  annotations as ticks. Click a span to zoom the plot to it. An unclosed
  span is hatched rather than hidden: a run that died mid-phase must not
  read like one that finished.
- **Signals** — canvas plot, drag to zoom, double-click to reset, crosshair
  with a per-channel value readout.
- **Events** — annotations in time order.

## Structure

```
src/data/     types, and the DataSource seam
src/lib/      scales, tick generation, formatting, palette
src/components/  RunHeader, Timeline, ChannelList, PlotCanvas
```

**`DataSource` is the seam everything hangs off.** Today the only
implementation is `UploadDataSource` (a file the user drops).
`ApiDataSource` is written but not wired up — it exists so the eventual
server contract is written down and so the interface has two implementers
rather than one. Swapping to a real backend should be one constructor
call in `App.tsx`.

`loadSeries` already takes a time range and a max-points hint that the
upload source ignores. That is deliberate: server-side decimation over a
requested window is the entire reason an API source would exist, and
retrofitting a parameter through a UI is much worse than ignoring one.

## Deliberately not here

- **Commanding.** The header says `read-only` and `Capabilities.canCommand`
  is false everywhere. Commanding a spacecraft from a browser needs an
  authority model, an audit trail and its own review — see the discussion
  of capability-scoped authority in the root README.
- **Auth.** When it lands, credentials belong in an httpOnly cookie set by
  the login flow, not in a token this app stores. A token in JS is a token
  in every XSS.
- **A generated type layer.** `src/data/types.ts` is hand-mirrored from the
  Python exporter. The `.proto` files are the source of truth and this
  should be generated from the descriptors — the same job as generating
  the exporter's `TOPIC_TYPES`. Until then, a blob format change has to be
  made in both places.
