//! Structured JSON event publisher — one event per line on stdout, matching
//! the `cog-person-count` publisher shape (ADR-100 runtime contract).
//!
//! Every flow event carries `synthetic` and `data_gated` so a downstream
//! consumer can never mistake a simulated or uncalibrated count for a validated
//! measurement.

use crate::flow::{Crossing, FlowEngine};
use serde::Serialize;
use serde_json::{json, Value};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Serialize)]
pub struct Event<'a> {
    pub ts: f64,
    pub level: &'a str,
    pub event: &'a str,
    pub fields: Value,
}

pub fn emit_event(ev: &Event<'_>) {
    if let Ok(line) = serde_json::to_string(ev) {
        println!("{line}");
    }
}

pub fn run_started(cog_id: &str, mode: &str, synthetic: bool, tick_ms: u64, mesh_dims: (usize, usize, f32)) {
    let (cols, rows, spacing) = mesh_dims;
    emit_event(&Event {
        ts: now_secs(),
        level: "info",
        event: "run.started",
        fields: json!({
            "cog": cog_id,
            "mode": mode,
            "tick_ms": tick_ms,
            "ap_grid": { "cols": cols, "rows": rows, "spacing_ft": spacing },
            // Honest disclosure — see module + lib docs.
            "synthetic": synthetic,
            "data_gated": true,
        }),
    });
}

pub fn health_ok(cog_id: &str, synthetic: bool) {
    emit_event(&Event {
        ts: now_secs(),
        level: "info",
        event: "health.ok",
        fields: json!({ "cog": cog_id, "synthetic": synthetic, "data_gated": true }),
    });
}

pub fn crossing(c: &Crossing, synthetic: bool) {
    emit_event(&Event {
        ts: now_secs(),
        level: "info",
        event: "package.crossing",
        fields: json!({
            "tick": c.tick,
            "gate": c.gate_name,
            "direction": c.direction.as_str(),
            "from_zone": c.from_zone_name,
            "to_zone": c.to_zone_name,
            "size": c.size.as_str(),
            "size_confidence": c.size_confidence,
            "peak_energy": c.peak_energy,
            "dwell_ticks": c.dwell_ticks,
            "synthetic": synthetic,
            "data_gated": true,
        }),
    });
}

/// Periodic roll-up of the running totals.
pub fn flow_summary(eng: &FlowEngine, tick: u64, throughput_per_min: f32, synthetic: bool) {
    let mesh = eng.mesh();
    let occupancy: Vec<Value> = mesh
        .zones
        .iter()
        .map(|z| json!({ "zone": z.name, "net": eng.occupancy[z.id] }))
        .collect();
    let gates: Vec<Value> = mesh
        .gates
        .iter()
        .map(|g| {
            json!({
                "gate": g.name,
                "forward": eng.gate_forward[g.id],
                "reverse": eng.gate_reverse[g.id],
            })
        })
        .collect();
    emit_event(&Event {
        ts: now_secs(),
        level: "info",
        event: "flow.summary",
        fields: json!({
            "tick": tick,
            "total_crossings": eng.total_crossings,
            "throughput_per_min": throughput_per_min,
            "size_counts": {
                "tote": eng.size_counts[0],
                "carton": eng.size_counts[1],
                "pallet": eng.size_counts[2],
                "forklift": eng.size_counts[3],
            },
            "gates": gates,
            "zone_net_flow": occupancy,
            "synthetic": synthetic,
            "data_gated": true,
        }),
    });
}

fn now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}
