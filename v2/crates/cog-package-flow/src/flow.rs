//! Flow-accounting engine — turns per-line firings into directional gate
//! crossings and maintains zone-to-zone package accounting.
//!
//! A gate is two adjacent sensing lines. One physical object crossing the gate
//! fires **both** lines; the *order* of their `start_tick`s gives direction
//! (fire low-line-then-high-line ⇒ travelling `+x`, `zone_lo → zone_hi`). We
//! pair a gate's two most-recent firings when their starts fall within
//! `match_window` ticks, apply a per-gate debounce so one object is not counted
//! twice, and update per-zone **net flow** (inbound − outbound crossings).

use crate::detector::{DetectorConfig, LineDetector, LineFiring, SizeClass};
use crate::geometry::WarehouseMesh;
use serde::Serialize;
use std::collections::VecDeque;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Direction {
    Forward,
    Reverse,
}

impl Direction {
    pub fn as_str(self) -> &'static str {
        match self {
            Direction::Forward => "forward",
            Direction::Reverse => "reverse",
        }
    }
}

/// One counted package-flow crossing at a gate.
#[derive(Debug, Clone, Serialize)]
pub struct Crossing {
    pub tick: u64,
    pub gate_id: usize,
    pub gate_name: String,
    pub direction: Direction,
    pub from_zone: usize,
    pub to_zone: usize,
    pub from_zone_name: String,
    pub to_zone_name: String,
    pub size: SizeClass,
    pub size_confidence: f32,
    pub peak_energy: f32,
    pub dwell_ticks: u64,
}

const RECENT_CAP: usize = 64;

/// Stateful engine. Feed it one motion-energy vector per link per tick; it
/// returns the crossings counted on that tick and accumulates running stats.
pub struct FlowEngine {
    mesh: WarehouseMesh,
    detectors: Vec<LineDetector>,
    line_to_gate: Vec<Option<usize>>,
    pending: Vec<Option<LineFiring>>,
    gate_last_cross: Vec<Option<u64>>,
    match_window: u64,
    min_gap: u64,
    line_energy: Vec<f32>,

    // ---- running statistics (all public for the snapshot) ----
    pub occupancy: Vec<i64>,
    pub gate_forward: Vec<u64>,
    pub gate_reverse: Vec<u64>,
    pub total_crossings: u64,
    /// Counts indexed `[tote, carton, pallet, forklift]`.
    pub size_counts: [u64; 4],
    recent: VecDeque<Crossing>,
}

impl FlowEngine {
    /// `match_window` = max start-tick separation to pair a gate's two lines;
    /// `min_gap` = min ticks between two counted crossings at the same gate.
    pub fn new(mesh: WarehouseMesh, det_cfg: DetectorConfig, match_window: u64, min_gap: u64) -> Self {
        let n_lines = mesh.lines.len();
        let n_gates = mesh.gates.len();
        let n_zones = mesh.zones.len();

        let detectors = (0..n_lines)
            .map(|id| LineDetector::new(id, det_cfg.clone()))
            .collect();

        // Each line belongs to at most one gate (first gate wins if shared).
        let mut line_to_gate = vec![None; n_lines];
        for g in &mesh.gates {
            for &l in &[g.line_lo, g.line_hi] {
                if line_to_gate[l].is_none() {
                    line_to_gate[l] = Some(g.id);
                }
            }
        }

        Self {
            detectors,
            line_to_gate,
            pending: vec![None; n_lines],
            gate_last_cross: vec![None; n_gates],
            match_window,
            min_gap,
            line_energy: vec![0.0; n_lines],
            occupancy: vec![0; n_zones],
            gate_forward: vec![0; n_gates],
            gate_reverse: vec![0; n_gates],
            total_crossings: 0,
            size_counts: [0; 4],
            recent: VecDeque::with_capacity(RECENT_CAP),
            mesh,
        }
    }

    pub fn with_defaults(mesh: WarehouseMesh) -> Self {
        // A gate's two tripwire lines sit one AP-gap (≈50 ft) apart, so at the
        // simulator's ~1.5 ft/tick a crossing fires them ~33 ticks apart — the
        // match window must comfortably exceed that. `min_gap` (15) rejects the
        // same object being re-counted while both lines are still settling.
        Self::new(mesh, DetectorConfig::default(), 60, 15)
    }

    pub fn mesh(&self) -> &WarehouseMesh {
        &self.mesh
    }
    pub fn line_energy(&self) -> &[f32] {
        &self.line_energy
    }
    pub fn recent(&self) -> impl DoubleEndedIterator<Item = &Crossing> {
        self.recent.iter()
    }

    /// Aggregate this tick's per-link energies onto sensing lines (max over each
    /// line's member links), run the per-line detectors, and match crossings.
    pub fn ingest_links(&mut self, tick: u64, link_energy: &[f32]) -> Vec<Crossing> {
        // 1. line energy = max over member links.
        for line in &self.mesh.lines {
            let e = line
                .link_ids
                .iter()
                .filter_map(|&lid| link_energy.get(lid).copied())
                .fold(0.0_f32, f32::max);
            self.line_energy[line.id] = e;
        }

        // 2. update detectors, gather firings.
        let mut firings = Vec::new();
        for line in &self.mesh.lines {
            let e = self.line_energy[line.id];
            if let Some(f) = self.detectors[line.id].update(tick, e) {
                firings.push(f);
            }
        }

        // 3. store + try to match the owning gate.
        let mut emitted = Vec::new();
        for f in firings {
            let line_id = f.line_id;
            self.pending[line_id] = Some(f);
            if let Some(g) = self.line_to_gate[line_id] {
                if let Some(cross) = self.try_match(g, tick) {
                    emitted.push(cross);
                }
            }
        }
        emitted
    }

    fn try_match(&mut self, gate_id: usize, tick: u64) -> Option<Crossing> {
        let gate = self.mesh.gates[gate_id].clone();
        let fl = self.pending[gate.line_lo].clone()?;
        let fh = self.pending[gate.line_hi].clone()?;

        let sep = fl.start_tick.abs_diff(fh.start_tick);
        if sep > self.match_window {
            return None; // not the same physical crossing (yet)
        }

        // Consume both firings regardless of debounce outcome so a suppressed
        // crossing cannot re-match on the next tick.
        self.pending[gate.line_lo] = None;
        self.pending[gate.line_hi] = None;

        if let Some(last) = self.gate_last_cross[gate_id] {
            if tick.saturating_sub(last) < self.min_gap {
                return None; // debounced — too soon after the last count
            }
        }
        self.gate_last_cross[gate_id] = Some(tick);

        // Direction from firing order; size from the stronger of the two firings.
        let direction = if fl.start_tick <= fh.start_tick {
            Direction::Forward
        } else {
            Direction::Reverse
        };
        let stronger = if fl.peak_energy >= fh.peak_energy { &fl } else { &fh };

        let (from_zone, to_zone) = match direction {
            Direction::Forward => (gate.zone_lo, gate.zone_hi),
            Direction::Reverse => (gate.zone_hi, gate.zone_lo),
        };

        // Accounting.
        self.occupancy[from_zone] -= 1;
        self.occupancy[to_zone] += 1;
        match direction {
            Direction::Forward => self.gate_forward[gate_id] += 1,
            Direction::Reverse => self.gate_reverse[gate_id] += 1,
        }
        self.total_crossings += 1;
        self.size_counts[size_index(stronger.size)] += 1;

        let cross = Crossing {
            tick,
            gate_id,
            gate_name: gate.name.clone(),
            direction,
            from_zone,
            to_zone,
            from_zone_name: self.mesh.zones[from_zone].name.clone(),
            to_zone_name: self.mesh.zones[to_zone].name.clone(),
            size: stronger.size,
            size_confidence: stronger.size_confidence,
            peak_energy: stronger.peak_energy,
            dwell_ticks: fl.dwell_ticks().max(fh.dwell_ticks()),
        };

        if self.recent.len() == RECENT_CAP {
            self.recent.pop_front();
        }
        self.recent.push_back(cross.clone());
        Some(cross)
    }
}

pub fn size_index(s: SizeClass) -> usize {
    match s {
        SizeClass::Tote => 0,
        SizeClass::Carton => 1,
        SizeClass::Pallet => 2,
        SizeClass::Forklift => 3,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a per-link energy vector that lights up every link in a given line.
    fn energize_line(mesh: &WarehouseMesh, line_id: usize, e: f32) -> Vec<f32> {
        let mut v = vec![0.0; mesh.links.len()];
        for &lid in &mesh.lines[line_id].link_ids {
            v[lid] = e;
        }
        v
    }

    fn zero(mesh: &WarehouseMesh) -> Vec<f32> {
        vec![0.0; mesh.links.len()]
    }

    /// Drive a forward crossing of gate 0: fire line_lo, then line_hi, then quiet.
    #[test]
    fn forward_crossing_is_counted_and_directional() {
        let mesh = WarehouseMesh::default_shopfloor();
        let g0 = mesh.gates[0].clone();
        let mut eng = FlowEngine::with_defaults(mesh.clone());

        let hot_lo = energize_line(&mesh, g0.line_lo, 0.7);
        let hot_hi = energize_line(&mesh, g0.line_hi, 0.7);
        let cold = zero(&mesh);

        let mut crossings = Vec::new();
        // lo rises first...
        crossings.extend(eng.ingest_links(0, &hot_lo));
        crossings.extend(eng.ingest_links(1, &hot_lo));
        // ...then hi joins (both active, lo started earlier)...
        crossings.extend(eng.ingest_links(2, &hot_hi));
        crossings.extend(eng.ingest_links(3, &hot_hi));
        // ...then everything goes quiet, both fire.
        crossings.extend(eng.ingest_links(4, &cold));
        crossings.extend(eng.ingest_links(5, &cold));

        assert_eq!(crossings.len(), 1, "exactly one crossing counted");
        let c = &crossings[0];
        assert_eq!(c.direction, Direction::Forward);
        assert_eq!(c.from_zone, g0.zone_lo);
        assert_eq!(c.to_zone, g0.zone_hi);
        assert_eq!(eng.gate_forward[0], 1);
        assert_eq!(eng.occupancy[g0.zone_lo], -1);
        assert_eq!(eng.occupancy[g0.zone_hi], 1);
    }

    #[test]
    fn reverse_order_yields_reverse_direction() {
        let mesh = WarehouseMesh::default_shopfloor();
        let g0 = mesh.gates[0].clone();
        let mut eng = FlowEngine::with_defaults(mesh.clone());
        let hot_lo = energize_line(&mesh, g0.line_lo, 0.7);
        let hot_hi = energize_line(&mesh, g0.line_hi, 0.7);
        let cold = zero(&mesh);

        // hi rises first this time → reverse.
        eng.ingest_links(0, &hot_hi);
        eng.ingest_links(1, &hot_hi);
        eng.ingest_links(2, &hot_lo);
        eng.ingest_links(3, &hot_lo);
        let mut crossings = eng.ingest_links(4, &cold);
        crossings.extend(eng.ingest_links(5, &cold));

        assert_eq!(crossings.len(), 1);
        assert_eq!(crossings[0].direction, Direction::Reverse);
        assert_eq!(eng.gate_reverse[0], 1);
    }

    #[test]
    fn a_lone_line_never_counts() {
        // Only one of the gate's two lines ever fires → no crossing.
        let mesh = WarehouseMesh::default_shopfloor();
        let g0 = mesh.gates[0].clone();
        let mut eng = FlowEngine::with_defaults(mesh.clone());
        let hot_lo = energize_line(&mesh, g0.line_lo, 0.7);
        let cold = zero(&mesh);
        for t in 0..3 {
            assert!(eng.ingest_links(t, &hot_lo).is_empty());
        }
        assert!(eng.ingest_links(3, &cold).is_empty());
        assert_eq!(eng.total_crossings, 0);
    }

    #[test]
    fn debounce_suppresses_a_too_soon_second_crossing() {
        let mesh = WarehouseMesh::default_shopfloor();
        let g0 = mesh.gates[0].clone();
        // Large min_gap so the second crossing lands inside the debounce window.
        let mut eng = FlowEngine::new(mesh.clone(), DetectorConfig::default(), 12, 50);
        let hot_lo = energize_line(&mesh, g0.line_lo, 0.7);
        let hot_hi = energize_line(&mesh, g0.line_hi, 0.7);
        let cold = zero(&mesh);

        let drive = |eng: &mut FlowEngine, base: u64| -> usize {
            let mut n = 0;
            n += eng.ingest_links(base, &hot_lo).len();
            n += eng.ingest_links(base + 1, &hot_hi).len();
            n += eng.ingest_links(base + 2, &cold).len();
            n
        };
        assert_eq!(drive(&mut eng, 0), 1);
        // Second crossing at tick 3 — within 50 ticks of the first → suppressed.
        assert_eq!(drive(&mut eng, 3), 0);
        assert_eq!(eng.total_crossings, 1);
    }
}
