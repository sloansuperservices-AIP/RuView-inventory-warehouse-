//! Runtime loops.
//!
//! * [`run_simulator`] drives the deterministic synthetic warehouse — the
//!   `simulate` and `serve` subcommands. `synthetic: true`.
//! * [`run_real`] polls a live `wifi-densepose-sensing-server` for per-node CSI,
//!   converts each node's frame-to-frame amplitude change into a motion-energy
//!   scalar, and feeds the same flow engine. The node→link mapping is
//!   deployment-specific and **must be calibrated** for real accuracy; the
//!   default identity mapping is a starting point, not a validated one.

use crate::flow::FlowEngine;
use crate::geometry::WarehouseMesh;
use crate::simulator::{SimConfig, Simulator};
use crate::{publisher, server};
use std::collections::VecDeque;
use std::time::Duration;
use tokio::time::sleep;

/// Parameters shared by both loops.
pub struct RunParams {
    pub tick_ms: u64,
    pub max_ticks: Option<u64>,
    pub emit_jsonl: bool,
    /// Emit a `flow.summary` every N ticks (0 = never).
    pub summary_every: u64,
}

struct ThroughputWindow {
    ticks: VecDeque<u64>,
    window: u64,
}
impl ThroughputWindow {
    fn new(tick_ms: u64) -> Self {
        Self {
            ticks: VecDeque::new(),
            window: (60_000 / tick_ms.max(1)).max(1),
        }
    }
    fn record(&mut self, tick: u64) {
        self.ticks.push_back(tick);
    }
    /// Crossings within the last minute → per-minute throughput.
    fn per_min(&mut self, tick: u64) -> f32 {
        while let Some(&front) = self.ticks.front() {
            if front + self.window <= tick {
                self.ticks.pop_front();
            } else {
                break;
            }
        }
        self.ticks.len() as f32
    }
}

/// Drive the synthetic warehouse. When `shared` is provided the latest snapshot
/// is published there for the dashboard.
pub async fn run_simulator(
    mesh: WarehouseMesh,
    sim_cfg: SimConfig,
    params: RunParams,
    shared: Option<server::SharedState>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut sim = Simulator::new(mesh.clone(), sim_cfg);
    let mut eng = FlowEngine::with_defaults(mesh);
    let mut tw = ThroughputWindow::new(params.tick_ms);

    let mut tick: u64 = 0;
    loop {
        let energy = sim.step(tick);
        let crossings = eng.ingest_links(tick, &energy);
        for c in &crossings {
            tw.record(tick);
            if params.emit_jsonl {
                publisher::crossing(c, true);
            }
        }
        let throughput = tw.per_min(tick);

        if let Some(sh) = &shared {
            let movers = sim.movers_xy();
            let snap = server::snapshot_json(&eng, &energy, &movers, tick, throughput, true);
            *sh.lock().await = snap;
        }
        if params.emit_jsonl && params.summary_every > 0 && tick > 0 && tick % params.summary_every == 0 {
            publisher::flow_summary(&eng, tick, throughput, true);
        }

        tick += 1;
        if let Some(max) = params.max_ticks {
            if tick >= max {
                break;
            }
        }
        sleep(Duration::from_millis(params.tick_ms)).await;
    }
    Ok(())
}

/// Per-node motion estimator: normalised frame-to-frame amplitude change.
#[derive(Default)]
struct MotionEstimator {
    prev: Vec<Vec<f32>>,
    scale: Vec<f32>,
}
impl MotionEstimator {
    /// Returns a motion-energy scalar in `[0, 1]` per node.
    fn update(&mut self, nodes: &[Vec<f32>]) -> Vec<f32> {
        if self.prev.len() != nodes.len() {
            self.prev = nodes.to_vec();
            self.scale = vec![1e-3; nodes.len()];
        }
        let mut out = vec![0.0_f32; nodes.len()];
        for (i, amp) in nodes.iter().enumerate() {
            let prev = &self.prev[i];
            let n = amp.len().min(prev.len());
            let motion = if n == 0 {
                0.0
            } else {
                amp.iter()
                    .zip(prev.iter())
                    .take(n)
                    .map(|(a, b)| (a - b).abs())
                    .sum::<f32>()
                    / n as f32
            };
            // Adaptive normaliser: track a slow-rising / slow-decaying peak.
            let s = &mut self.scale[i];
            *s = (*s * 0.995).max(motion).max(1e-3);
            out[i] = (motion / *s).clamp(0.0, 1.0);
            self.prev[i] = amp.clone();
        }
        out
    }
}

/// Poll a sensing-server and drive the flow engine off real CSI motion.
pub async fn run_real(
    mesh: WarehouseMesh,
    sensing_url: String,
    params: RunParams,
    shared: Option<server::SharedState>,
) -> Result<(), Box<dyn std::error::Error>> {
    let n_links = mesh.links.len();
    let mut eng = FlowEngine::with_defaults(mesh);
    let mut est = MotionEstimator::default();
    let mut tw = ThroughputWindow::new(params.tick_ms);

    let mut tick: u64 = 0;
    loop {
        match fetch_nodes(&sensing_url).await {
            Ok(nodes) => {
                let node_energy = est.update(&nodes);
                // Deployment-specific mapping — identity (node k → link k) by
                // default. Calibrate this to your AP topology for real accuracy.
                let mut link_energy = vec![0.0_f32; n_links];
                for (k, &e) in node_energy.iter().enumerate() {
                    if k < n_links {
                        link_energy[k] = e;
                    }
                }
                let crossings = eng.ingest_links(tick, &link_energy);
                for c in &crossings {
                    tw.record(tick);
                    if params.emit_jsonl {
                        publisher::crossing(c, false);
                    }
                }
                let throughput = tw.per_min(tick);
                if let Some(sh) = &shared {
                    let snap = server::snapshot_json(&eng, &link_energy, &[], tick, throughput, false);
                    *sh.lock().await = snap;
                }
                if params.emit_jsonl && params.summary_every > 0 && tick > 0 && tick % params.summary_every == 0 {
                    publisher::flow_summary(&eng, tick, throughput, false);
                }
                tick += 1;
            }
            Err(e) => tracing::warn!(error = %e, "sensing-server fetch failed"),
        }
        if let Some(max) = params.max_ticks {
            if tick >= max {
                break;
            }
        }
        sleep(Duration::from_millis(params.tick_ms)).await;
    }
    Ok(())
}

/// Fetch `nodes[].amplitude[]` from a sensing-server snapshot endpoint.
async fn fetch_nodes(url: &str) -> Result<Vec<Vec<f32>>, Box<dyn std::error::Error>> {
    let url = url.to_string();
    let body = tokio::task::spawn_blocking(move || -> Result<String, ureq::Error> {
        Ok(ureq::get(&url).call()?.into_string()?)
    })
    .await??;
    let json: serde_json::Value = serde_json::from_str(&body)?;
    let snapshot = json.get("snapshot").unwrap_or(&json);
    let nodes = snapshot
        .get("nodes")
        .and_then(|v| v.as_array())
        .ok_or("missing nodes[]")?;
    let mut out = Vec::with_capacity(nodes.len());
    for n in nodes {
        if let Some(amp) = n.get("amplitude").and_then(|v| v.as_array()) {
            out.push(
                amp.iter()
                    .filter_map(|v| v.as_f64().map(|f| f as f32))
                    .collect(),
            );
        }
    }
    if out.is_empty() {
        return Err("no node amplitudes in snapshot".into());
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_estimator_flags_change_not_steady_state() {
        let mut est = MotionEstimator::default();
        // First call primes the previous frame.
        let steady = vec![vec![1.0, 1.0, 1.0]];
        est.update(&steady);
        // Same frame again → ~no motion.
        let e0 = est.update(&steady);
        assert!(e0[0] < 0.5);
        // A big change → high motion energy.
        let moved = vec![vec![5.0, 0.0, 5.0]];
        let e1 = est.update(&moved);
        assert!(e1[0] > 0.5, "a big amplitude change should read as motion");
    }

    #[test]
    fn throughput_window_expires_old_crossings() {
        let mut tw = ThroughputWindow::new(100); // window = 600 ticks
        tw.record(0);
        tw.record(1);
        assert_eq!(tw.per_min(1), 2.0);
        // Far in the future — both fall out of the 600-tick window.
        assert_eq!(tw.per_min(1000), 0.0);
    }
}
