//! Deterministic synthetic warehouse — drives the mesh with zero hardware.
//!
//! `synthetic: true`, `data_gated: true`. This makes **no accuracy claim**; it
//! exists to exercise the geometry → detector → flow pipeline end-to-end and to
//! feed the live dashboard so the concept is demoable without a shop floor.
//!
//! Model: "movers" (totes / cartons / pallets / forklifts) enter at an edge and
//! travel along a row. Each mover paints a Gaussian bump of **motion energy**
//! onto the links whose Fresnel zone it passes through — sharp in `x` (a couple
//! of ticks as it crosses a tripwire plane) and broad-but-decaying in `y` (its
//! own row lights up, neighbours barely). The RNG is a seeded SplitMix64 so a
//! given `seed` replays byte-identically (no wall-clock, per this repo's rules).

use crate::detector::SizeClass;
use crate::geometry::WarehouseMesh;

/// Fresnel-bump spread in feet. `x` is tight (tripwire plane), `y` is ~half the
/// AP spacing so a mover mainly lights the link in its own row.
const SIGMA_X_FT: f32 = 12.0;
const SIGMA_Y_FT: f32 = 22.0;

#[derive(Debug, Clone)]
pub struct SimConfig {
    pub seed: u64,
    /// Mean ticks between spawns (actual gap is jittered ±50%).
    pub spawn_every_ticks: u64,
    /// Travel speed along the row, feet per tick.
    pub speed_ft_per_tick: f32,
    /// Fraction of movers that travel `-x` (returns / put-backs).
    pub reverse_fraction: f32,
    /// Baseline energy present on every link (sensor/thermal noise floor).
    pub noise_floor: f32,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            seed: 0x00C0_FFEE_D15E_A5E5,
            spawn_every_ticks: 22,
            speed_ft_per_tick: 1.5,
            reverse_fraction: 0.18,
            noise_floor: 0.03,
        }
    }
}

#[derive(Debug, Clone)]
struct Mover {
    x_ft: f32,
    y_ft: f32,
    vx: f32,
    base_energy: f32,
    size: SizeClass,
}

pub struct Simulator {
    mesh: WarehouseMesh,
    cfg: SimConfig,
    rng: u64,
    movers: Vec<Mover>,
    next_spawn: u64,
    pub spawned: u64,
}

impl Simulator {
    pub fn new(mesh: WarehouseMesh, cfg: SimConfig) -> Self {
        let rng = cfg.seed | 1; // avoid an all-zero SplitMix state
        let first = cfg.spawn_every_ticks / 2;
        Self {
            mesh,
            cfg,
            rng,
            movers: Vec::new(),
            next_spawn: first,
            spawned: 0,
        }
    }

    pub fn mesh(&self) -> &WarehouseMesh {
        &self.mesh
    }

    /// Advance one tick and return per-link motion energy (indexed by link id).
    pub fn step(&mut self, tick: u64) -> Vec<f32> {
        if tick >= self.next_spawn {
            self.spawn(tick);
            let jitter = self.rand_range(-0.5, 0.5);
            let gap = (self.cfg.spawn_every_ticks as f32 * (1.0 + jitter)).max(4.0);
            self.next_spawn = tick + gap as u64;
        }

        let len = self.mesh.floor_len_ft();
        for m in &mut self.movers {
            m.x_ft += m.vx;
        }
        // Drop movers that have left the floor (with a margin so their tail fades).
        self.movers
            .retain(|m| m.x_ft > -SIGMA_X_FT * 2.0 && m.x_ft < len + SIGMA_X_FT * 2.0);

        // Paint energy onto every link. Pre-generate per-link noise first so the
        // RNG (&mut self) isn't borrowed while iterating the mesh (&self).
        let n_links = self.mesh.links.len();
        let noise: Vec<f32> = (0..n_links)
            .map(|_| self.cfg.noise_floor + self.rand_range(0.0, 0.02))
            .collect();
        let mut energy = vec![0.0_f32; n_links];
        for (i, link) in self.mesh.links.iter().enumerate() {
            let mut e = noise[i];
            for m in &self.movers {
                let dx = m.x_ft - link.mid_x;
                let dy = m.y_ft - link.mid_y;
                let s = (-(dx * dx) / (2.0 * SIGMA_X_FT * SIGMA_X_FT)).exp()
                    * (-(dy * dy) / (2.0 * SIGMA_Y_FT * SIGMA_Y_FT)).exp();
                e += m.base_energy * s;
            }
            energy[link.id] = e.clamp(0.0, 1.0);
        }
        energy
    }

    /// Live mover positions for the dashboard: `(x_ft, y_ft, size)`.
    pub fn movers_xy(&self) -> Vec<(f32, f32, &'static str)> {
        self.movers
            .iter()
            .map(|m| (m.x_ft, m.y_ft, m.size.as_str()))
            .collect()
    }

    fn spawn(&mut self, _tick: u64) {
        let reverse = self.rand_unit() < self.cfg.reverse_fraction;
        let row = (self.rand_unit() * self.mesh.rows as f32) as usize;
        let row = row.min(self.mesh.rows.saturating_sub(1));
        let y_ft = row as f32 * self.mesh.spacing_ft;

        // Size mix weighted toward cartons/totes; base energy matches the
        // detector's size cuts (tote<0.45<carton<0.62<pallet<0.80<forklift).
        let (size, base) = match self.rand_unit() {
            r if r < 0.40 => (SizeClass::Carton, 0.55),
            r if r < 0.68 => (SizeClass::Tote, 0.37),
            r if r < 0.90 => (SizeClass::Pallet, 0.72),
            _ => (SizeClass::Forklift, 0.92),
        };
        let base_energy = (base + self.rand_range(-0.03, 0.03)).clamp(0.05, 1.0);
        let speed = self.cfg.speed_ft_per_tick
            * if matches!(size, SizeClass::Forklift) { 1.4 } else { 1.0 };

        let (x_ft, vx) = if reverse {
            (self.mesh.floor_len_ft(), -speed)
        } else {
            (0.0, speed)
        };

        self.movers.push(Mover {
            x_ft,
            y_ft,
            vx,
            base_energy,
            size,
        });
        self.spawned += 1;
    }

    // ---- deterministic SplitMix64 PRNG ----
    fn next_u64(&mut self) -> u64 {
        self.rng = self.rng.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.rng;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform `[0, 1)`.
    fn rand_unit(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u32 << 24) as f32
    }
    fn rand_range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + self.rand_unit() * (hi - lo)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_deterministic_for_a_seed() {
        let run = || {
            let mut s = Simulator::new(WarehouseMesh::default_shopfloor(), SimConfig::default());
            let mut acc = 0.0f64;
            for t in 0..500 {
                acc += s.step(t).iter().map(|&e| e as f64).sum::<f64>();
            }
            (acc, s.spawned)
        };
        let (a, sa) = run();
        let (b, sb) = run();
        assert_eq!(sa, sb, "same seed spawns the same count");
        assert!((a - b).abs() < 1e-9, "same seed replays identical energy");
    }

    #[test]
    fn movers_actually_traverse_and_drive_crossings() {
        use crate::flow::FlowEngine;
        let mesh = WarehouseMesh::default_shopfloor();
        let mut sim = Simulator::new(mesh.clone(), SimConfig::default());
        let mut eng = FlowEngine::with_defaults(mesh);
        let mut total = 0u64;
        for t in 0..2000 {
            let e = sim.step(t);
            total += eng.ingest_links(t, &e).len() as u64;
        }
        assert!(sim.spawned > 10, "simulator should spawn movers");
        assert!(total > 0, "movers crossing gates should produce crossings");
        assert_eq!(eng.total_crossings, total);
    }

    #[test]
    fn energy_is_bounded() {
        let mut s = Simulator::new(WarehouseMesh::default_shopfloor(), SimConfig::default());
        for t in 0..300 {
            for &e in &s.step(t) {
                assert!((0.0..=1.0).contains(&e), "energy {e} out of [0,1]");
            }
        }
    }
}
