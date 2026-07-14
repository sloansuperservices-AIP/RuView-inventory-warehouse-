# ADR-266: WiFi-CSI Package-Flow Tracker (directional RF tripwires over an existing AP grid)

- **Status:** Proposed
- **Date:** 2026-07-14
- **Deciders:** ruv
- **Motivating request:** Track and count packages/pallets on a shop floor using the **existing** WiFi access points already spread roughly every 50 ft — no new tags, cameras, or barcodes.
- **Related:** ADR-100 (Cog packaging), ADR-103 (`cog-person-count` — the crate this mirrors), ADR-029/030 (RuvSense multistatic sensing), ADR-135 (empty-room calibration), ADR-028 (capability audit / honest disclosure discipline)

## Context

A packaging-and-logistics shop floor already has WiFi APs on a grid at ~50 ft
spacing. The ask: turn that infrastructure into a package tracker/counter.

The physics has to lead the design:

- **A passive cardboard box sitting still is essentially invisible to WiFi CSI
  at range.** We cannot image it, read a barcode, or identify an individual SKU
  from the RF field. Anyone who claims otherwise on 1×1 SISO APs is selling
  something.
- **What the AP mesh *can* do reliably is detect motion.** Each pair of
  neighbouring APs is a **bistatic link** whose Channel State Information is
  perturbed when something with mass and motion passes through the Fresnel zone
  between the two radios. This is the same signal `wifi-densepose-signal`
  already exploits for presence/pose.

So the honest, physically-grounded product is **not** a shelf-inventory imager.
It is a **flow counter**: count *crossing events* through the aisles and dock
doors, infer direction, and do zone-to-zone accounting — exactly the primitive a
warehouse needs to answer "how many packages moved from Receiving into Storage
this hour, and which way is the forklift traffic going?"

Five things we can compose that we already have or can build cheaply:

1. A grid of APs with known positions (deployment fact).
2. Per-link CSI motion energy (temporal amplitude variance — a subset of what
   `ruvsense` computes today).
3. A Schmitt-trigger gating discipline (mirrors `ruvsense::coherence_gate`).
4. The Cognitum Cog packaging + JSONL runtime contract (ADR-100).
5. The `cog-person-count` crate as a structural template (ADR-103).

## Decision

Ship a new Cognitum Cog, **`cog-package-flow`**, that models the AP grid as a
mesh of **directional RF tripwires** and does package **flow accounting**.

### The core idea

```
   row 2   AP────AP────AP────AP────AP────AP────AP
           │  h  │     │     │     │     │     │      h = horizontal bistatic link
   row 1   AP────AP────AP────AP────AP────AP────AP
           │     │     │     │     │     │     │
   row 0   AP────AP────AP────AP────AP────AP────AP
          col0  col1  col2  col3  col4  col5  col6
                └─L0──┘                              L_g = "sensing line" g
   zones : [ Receiving ][ Storage-A ][ Storage-B ][Ship]
   gates :          Gate         Gate         Gate
```

- All horizontal links sharing an x-position form a **sensing line** — a vertical
  tripwire *plane* at `x = (gap + 0.5)·spacing`.
- Two adjacent sensing lines form a **gate**. One object crossing the gate fires
  **both** lines; the *order* of the firings gives the crossing **direction**
  (fire the low line then the high line ⇒ travelling `+x`) — exactly like an
  IR beam-break people-counter, but built from CSI motion energy.
- A **zone** is a band of floor between gates. Every directional crossing moves
  one unit of inferred occupancy from one zone to the next → a running
  **package-flow count** and per-zone net throughput, from the *existing*
  infrastructure.

### Architecture (v0.1.0)

```
per-link CSI motion energy
        │  aggregate max over each sensing line's member links
        ▼
  LineDetector (Schmitt trigger: enter/exit hysteresis + min-ticks)  ── per line
        │  LineFiring{ start_tick, peak_energy, size_class, size_confidence }
        ▼
  FlowEngine  ── pair a gate's two most-recent firings within match_window,
        │         debounce per gate, derive direction from firing order,
        │         update zone net-flow + per-gate forward/reverse counts
        ▼
  package.crossing / flow.summary  (JSONL)  +  /api/flow/state (dashboard)
```

- **Size class** (tote / carton / pallet / forklift) is inferred from motion-energy
  magnitude and dwell. It is a **heuristic, not a validated classifier** — every
  event carries a margin-based `size_confidence` and the whole stream is flagged
  `data_gated: true` until calibrated against labelled crossings on real hardware.
- **Runtime paths:**
  - `serve` / `simulate` — a **deterministic** (seeded SplitMix64) synthetic
    warehouse that paints Gaussian motion bumps onto the mesh as movers traverse
    it. `synthetic: true`. Proves the algorithm and drives the live dashboard
    with **zero hardware**; makes **no accuracy claim**.
  - `run` — polls a live `wifi-densepose-sensing-server`, converts each node's
    frame-to-frame amplitude change into a motion-energy scalar, and feeds the
    same `FlowEngine`. The node→link mapping is deployment-specific and **must be
    calibrated**; the default identity mapping is a starting point, not a
    validated one.
- **Dashboard** — a single self-contained HTML file (vanilla JS + SVG, no CDN)
  served by Axum on `/`, polling `/api/flow/state`. Renders the floor: AP nodes,
  links coloured by live energy, gates with forward/reverse counters, zones with
  net flow, moving package dots, and a recent-crossings feed.

### Default reference floor

7-column × 3-row AP grid at 50 ft spacing (300 ft × 100 ft), four zones
(Receiving → Storage-A → Storage-B → Shipping) connected by three gates, each a
non-overlapping adjacent sensing-line pair.

## Consequences

**Positive**

- Uses infrastructure the customer already owns; no tags, cameras, or barcodes.
- Answers the real logistics question (throughput / directional flow / zone
  balance) rather than the unattainable one (per-SKU imaging).
- Pure-Rust, edge-deployable, self-contained; demoable with `serve` on a laptop.
- Follows the established Cog contract, so it drops into the same packaging,
  manifest, and JSONL-consumer tooling as `cog-person-count`.

**Negative / limits (stated plainly)**

- Counts **flow events**, not static shelf inventory. A box that never moves is
  never counted.
- Cannot identify or distinguish individual packages; "size class" is coarse and
  uncalibrated (`data_gated`).
- Two objects crossing a gate within `min_gap` ticks, or side-by-side in
  different rows within the same sensing line, can under-count; overlapping
  movers saturate energy (→ low `size_confidence`, honestly flagged).
- The real (`run`) path requires per-site calibration of the node→link map and
  the motion-energy normaliser before any accuracy claim is legitimate.

**Follow-ups**

- Collect labelled crossings (a manual clicker or a temporary camera at one dock
  door) to calibrate size cuts and measure crossing precision/recall → lift
  `data_gated`.
- Multi-row disambiguation (per-link, not per-line, firing) to separate two
  objects crossing abreast.
- Optional fusion with BLE/RFID at dock doors for identity where it matters,
  keeping CSI for tag-free interior flow.
