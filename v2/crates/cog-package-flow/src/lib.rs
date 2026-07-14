//! `cog-package-flow` — WiFi-CSI package-flow tracker (ADR-266).
//!
//! ## The idea
//!
//! A shop floor already has WiFi access points spread roughly every 50 ft.
//! Each pair of neighbouring APs forms a **bistatic link**: the Channel State
//! Information (CSI) on that link is perturbed whenever something with mass and
//! motion passes through the Fresnel zone between the two radios. We do **not**
//! try to image or identify a package — cardboard sitting still is essentially
//! invisible to WiFi at range. Instead we treat the AP grid as a mesh of
//! **directional RF tripwires** and count *flow events*:
//!
//!   * all the horizontal links sharing an x-position form a vertical
//!     **sensing line** (a "tripwire plane"),
//!   * two adjacent sensing lines form a **gate** — the order in which the two
//!     lines fire gives the crossing *direction* (exactly like a beam-break
//!     people-counter, but built from CSI motion energy instead of IR),
//!   * a **zone** is a band of floor between gates; every directional crossing
//!     moves one unit of inferred occupancy from one zone to the next.
//!
//! That yields a running **package-flow count** and per-zone throughput from
//! the *existing* infrastructure — no new tags, no cameras, no barcodes.
//!
//! ## Honesty (this repo's house style — see ADR-028/103)
//!
//! * We count **crossing events**, not a static inventory. A box that never
//!   moves is never counted; the number is *flow through a gate*, not *stock on
//!   a shelf*.
//! * We infer a coarse **size class** (tote / carton / pallet / forklift) from
//!   motion-energy magnitude and dwell — a heuristic, **not** a validated
//!   classifier. Every event carries `size_confidence` and the whole stream is
//!   flagged `data_gated: true` until it is calibrated against labelled crossings
//!   on real hardware.
//! * The default `simulate`/`serve` path is explicitly `synthetic: true`. It
//!   proves the algorithm and drives the dashboard end-to-end with zero
//!   hardware; it makes **no accuracy claim**.
//!
//! ## Layout (mirrors `cog-person-count`, ADR-100 runtime contract)
//!
//! | module        | responsibility                                            |
//! |---------------|-----------------------------------------------------------|
//! | `geometry`    | AP grid, bistatic links, sensing lines, gates, zones      |
//! | `detector`    | per-line motion-energy gate with hysteresis + size class  |
//! | `flow`        | directional crossing matching, debounce, zone accounting  |
//! | `simulator`   | deterministic synthetic warehouse (movers across the mesh)|
//! | `runtime`     | poll sensing-server (real) or drive the simulator         |
//! | `publisher`   | one JSONL event per line on stdout, honest disclosure      |
//! | `server`      | Axum `/api/flow/state` + self-contained live dashboard     |

pub mod detector;
pub mod flow;
pub mod geometry;
pub mod publisher;
pub mod runtime;
pub mod server;
pub mod simulator;

pub const COG_ID: &str = "package-flow";
pub const COG_VERSION: &str = env!("CARGO_PKG_VERSION");
