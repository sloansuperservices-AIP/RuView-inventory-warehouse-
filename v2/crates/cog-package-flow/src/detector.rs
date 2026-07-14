//! Per-sensing-line motion detection.
//!
//! Each sensing line reports a scalar **motion energy** per tick — the
//! aggregate CSI perturbation across its member links (in the real path this is
//! temporal variance of subcarrier amplitude; in the simulator it is the
//! synthesised bump of a passing mover). This module turns that continuous
//! signal into discrete **firing events** with a Schmitt-trigger hysteresis
//! gate, and estimates a coarse size class from the event's peak energy and
//! dwell. It mirrors the accept/reject spirit of
//! `wifi-densepose-signal::ruvsense::coherence_gate` but at package scale.

use serde::Serialize;

/// Coarse size class inferred from motion-energy magnitude and dwell time.
///
/// **Heuristic, not a validated classifier.** The thresholds are physically
/// motivated (a forklift perturbs far more of the Fresnel volume, for longer,
/// than a hand-carried tote) but uncalibrated — every event carries a
/// `size_confidence` and the stream is `data_gated` until labelled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum SizeClass {
    Tote,
    Carton,
    Pallet,
    Forklift,
}

impl SizeClass {
    pub fn as_str(self) -> &'static str {
        match self {
            SizeClass::Tote => "tote",
            SizeClass::Carton => "carton",
            SizeClass::Pallet => "pallet",
            SizeClass::Forklift => "forklift",
        }
    }
}

/// Thresholds controlling the hysteresis gate and size mapping.
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Energy above which an idle line starts a firing.
    pub enter: f32,
    /// Energy below which an active line ends its firing (must be < `enter`).
    pub exit: f32,
    /// Peak-energy cutoffs for `[carton, pallet, forklift]` (tote is below the first).
    pub size_cuts: [f32; 3],
    /// Ticks a firing must persist to be emitted (rejects single-tick noise spikes).
    pub min_ticks: u32,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        // Motion energy is normalised to roughly [0, 1] by the simulator and the
        // real-path variance normaliser, so these live in that band.
        Self {
            enter: 0.30,
            exit: 0.18,
            size_cuts: [0.45, 0.62, 0.80],
            min_ticks: 2,
        }
    }
}

/// A completed line firing — one physical thing passed this tripwire plane.
#[derive(Debug, Clone)]
pub struct LineFiring {
    pub line_id: usize,
    /// Tick at which the firing crossed the `enter` threshold.
    pub start_tick: u64,
    /// Tick at which it fell back below `exit`.
    pub end_tick: u64,
    pub peak_energy: f32,
    pub size: SizeClass,
    /// `[0,1]` distance-from-boundary margin — how cleanly the peak lands inside
    /// its size bucket. Low near a bucket edge.
    pub size_confidence: f32,
}

impl LineFiring {
    pub fn dwell_ticks(&self) -> u64 {
        self.end_tick.saturating_sub(self.start_tick).max(1)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum State {
    Idle,
    Active,
}

/// Schmitt-trigger detector for a single sensing line.
#[derive(Debug, Clone)]
pub struct LineDetector {
    pub line_id: usize,
    cfg: DetectorConfig,
    state: State,
    start_tick: u64,
    peak: f32,
    ticks_active: u32,
    /// Most recent energy, kept for the dashboard.
    pub last_energy: f32,
}

impl LineDetector {
    pub fn new(line_id: usize, cfg: DetectorConfig) -> Self {
        Self {
            line_id,
            cfg,
            state: State::Idle,
            start_tick: 0,
            peak: 0.0,
            ticks_active: 0,
            last_energy: 0.0,
        }
    }

    pub fn is_active(&self) -> bool {
        matches!(self.state, State::Active)
    }

    /// Feed one tick of motion energy. Returns `Some(firing)` on the tick the
    /// line falls back to idle after a valid (≥ `min_ticks`) activation.
    pub fn update(&mut self, tick: u64, energy: f32) -> Option<LineFiring> {
        self.last_energy = energy;
        match self.state {
            State::Idle => {
                if energy >= self.cfg.enter {
                    self.state = State::Active;
                    self.start_tick = tick;
                    self.peak = energy;
                    self.ticks_active = 1;
                }
                None
            }
            State::Active => {
                self.peak = self.peak.max(energy);
                self.ticks_active += 1;
                if energy < self.cfg.exit {
                    self.state = State::Idle;
                    if self.ticks_active >= self.cfg.min_ticks {
                        let (size, size_confidence) = classify(self.peak, &self.cfg.size_cuts);
                        return Some(LineFiring {
                            line_id: self.line_id,
                            start_tick: self.start_tick,
                            end_tick: tick,
                            peak_energy: self.peak,
                            size,
                            size_confidence,
                        });
                    }
                }
                None
            }
        }
    }
}

/// Map peak energy to a size class plus a margin-based confidence.
fn classify(peak: f32, cuts: &[f32; 3]) -> (SizeClass, f32) {
    let (size, lo, hi) = if peak < cuts[0] {
        (SizeClass::Tote, 0.0, cuts[0])
    } else if peak < cuts[1] {
        (SizeClass::Carton, cuts[0], cuts[1])
    } else if peak < cuts[2] {
        (SizeClass::Pallet, cuts[1], cuts[2])
    } else {
        (SizeClass::Forklift, cuts[2], 1.0)
    };
    // Confidence = how far the peak sits from the nearest bucket edge, scaled to
    // the bucket width. 1.0 at bucket centre, → 0 at an edge.
    let width = (hi - lo).max(1e-3);
    let centre = 0.5 * (lo + hi);
    let margin = 1.0 - (2.0 * (peak - centre).abs() / width);
    (size, margin.clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(det: &mut LineDetector, energies: &[f32]) -> Vec<LineFiring> {
        let mut out = Vec::new();
        for (i, &e) in energies.iter().enumerate() {
            if let Some(f) = det.update(i as u64, e) {
                out.push(f);
            }
        }
        out
    }

    #[test]
    fn hysteresis_needs_enter_then_exit() {
        let mut d = LineDetector::new(0, DetectorConfig::default());
        // Rises above enter (0.30), stays high, then falls below exit (0.18).
        let firings = run(&mut d, &[0.1, 0.5, 0.6, 0.4, 0.1]);
        assert_eq!(firings.len(), 1);
        assert!((firings[0].peak_energy - 0.6).abs() < 1e-6);
        assert_eq!(firings[0].start_tick, 1);
    }

    #[test]
    fn between_thresholds_does_not_retrigger() {
        // Energy that stays in the (exit, enter) band after activation must not
        // drop out — that is the whole point of the Schmitt gate.
        let mut d = LineDetector::new(0, DetectorConfig::default());
        let firings = run(&mut d, &[0.5, 0.25, 0.25, 0.05]);
        assert_eq!(firings.len(), 1);
        assert_eq!(firings[0].end_tick, 3);
    }

    #[test]
    fn single_tick_spike_is_rejected() {
        let mut d = LineDetector::new(0, DetectorConfig { min_ticks: 3, ..Default::default() });
        let firings = run(&mut d, &[0.05, 0.9, 0.05]);
        assert!(firings.is_empty(), "a 1-tick spike must not emit a firing");
    }

    #[test]
    fn size_classes_scale_with_peak() {
        let cuts = DetectorConfig::default().size_cuts;
        assert_eq!(classify(0.20, &cuts).0, SizeClass::Tote);
        assert_eq!(classify(0.50, &cuts).0, SizeClass::Carton);
        assert_eq!(classify(0.70, &cuts).0, SizeClass::Pallet);
        assert_eq!(classify(0.95, &cuts).0, SizeClass::Forklift);
    }

    #[test]
    fn size_confidence_peaks_at_bucket_centre() {
        let cuts = DetectorConfig::default().size_cuts;
        // Carton bucket is [0.45, 0.62); centre ≈ 0.535.
        let (_, c_centre) = classify(0.535, &cuts);
        let (_, c_edge) = classify(0.46, &cuts);
        assert!(c_centre > c_edge);
        assert!(c_centre > 0.9);
    }
}
