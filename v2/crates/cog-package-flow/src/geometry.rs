//! Warehouse RF-sensing geometry — the spatial model that turns an existing
//! grid of WiFi access points into directional tripwires.
//!
//! Coordinates are in **feet**, origin at the bottom-left AP, `+x` pointing
//! down the length of the floor (receiving → shipping), `+y` across it.
//!
//! ```text
//!   row 2   AP────AP────AP────AP────AP────AP────AP
//!           │  h  │     │     │     │     │     │      h = horizontal link
//!   row 1   AP────AP────AP────AP────AP────AP────AP      (a bistatic pair)
//!           │     │     │     │     │     │     │
//!   row 0   AP────AP────AP────AP────AP────AP────AP
//!          col0  col1  col2  col3  col4  col5  col6
//!                └──L0──┘                              L_g = sensing line in
//!                                                            column-gap g
//!   zones:  [ Receiving ][ Storage-A ][ Storage-B ][Ship]
//!   gates :          Gate@G1     Gate@G2     Gate@G3
//! ```
//!
//! A **sensing line** `L_g` is the set of horizontal links spanning column-gap
//! `g` (one per row); they all share the same midpoint-x, so together they act
//! as one vertical tripwire plane at `x = (g + 0.5) * spacing`. A **gate** pairs
//! two neighbouring lines so the firing *order* yields crossing direction.

use serde::Serialize;

/// Access point node in the mesh.
#[derive(Debug, Clone, Serialize)]
pub struct ApNode {
    pub id: usize,
    pub col: usize,
    pub row: usize,
    pub x_ft: f32,
    pub y_ft: f32,
}

/// A bistatic link between two neighbouring APs. Only horizontal links (same
/// row, adjacent columns) participate in tripwire lines; we keep the vertical
/// links too because they contribute to the mesh's motion-energy picture and
/// the dashboard renders them.
#[derive(Debug, Clone, Serialize)]
pub struct Link {
    pub id: usize,
    pub a: usize,
    pub b: usize,
    pub mid_x: f32,
    pub mid_y: f32,
    /// `true` when the link spans two columns in the same row (participates in a
    /// vertical sensing line); `false` for a vertical (same-column) link.
    pub horizontal: bool,
    /// Column-gap index this link's tripwire plane sits in (`col.min`), valid
    /// only when `horizontal`.
    pub gap: usize,
}

/// A vertical tripwire plane: all horizontal links in one column-gap.
#[derive(Debug, Clone, Serialize)]
pub struct SensingLine {
    pub id: usize,
    pub gap: usize,
    pub x_ft: f32,
    pub link_ids: Vec<usize>,
}

/// A floor zone — a band of columns between two gates (or a boundary).
#[derive(Debug, Clone, Serialize)]
pub struct Zone {
    pub id: usize,
    pub name: String,
    /// Inclusive AP-column range `[col_lo, col_hi]` covered by the zone.
    pub col_lo: usize,
    pub col_hi: usize,
}

/// A directional gate built from two adjacent sensing lines. A crossing that
/// fires `line_lo` *then* `line_hi` is travelling `+x` (`forward`), moving one
/// unit from `zone_lo` → `zone_hi`; the reverse order moves `zone_hi` → `zone_lo`.
#[derive(Debug, Clone, Serialize)]
pub struct Gate {
    pub id: usize,
    pub name: String,
    /// Sensing line on the `-x` (low) side.
    pub line_lo: usize,
    /// Sensing line on the `+x` (high) side.
    pub line_hi: usize,
    pub zone_lo: usize,
    pub zone_hi: usize,
    pub x_ft: f32,
}

/// The complete sensing geometry for a floor.
#[derive(Debug, Clone, Serialize)]
pub struct WarehouseMesh {
    pub spacing_ft: f32,
    pub cols: usize,
    pub rows: usize,
    pub nodes: Vec<ApNode>,
    pub links: Vec<Link>,
    pub lines: Vec<SensingLine>,
    pub zones: Vec<Zone>,
    pub gates: Vec<Gate>,
}

impl WarehouseMesh {
    /// Build the raw grid: `cols × rows` APs at `spacing_ft`, all
    /// nearest-neighbour bistatic links, and the vertical sensing lines. Zones
    /// and gates are added by the caller (see [`default_shopfloor`]).
    pub fn grid(cols: usize, rows: usize, spacing_ft: f32) -> Self {
        assert!(cols >= 2 && rows >= 1, "need at least a 2×1 AP grid");
        let mut nodes = Vec::with_capacity(cols * rows);
        let node_id = |c: usize, r: usize| r * cols + c;
        for r in 0..rows {
            for c in 0..cols {
                nodes.push(ApNode {
                    id: node_id(c, r),
                    col: c,
                    row: r,
                    x_ft: c as f32 * spacing_ft,
                    y_ft: r as f32 * spacing_ft,
                });
            }
        }

        let mut links = Vec::new();
        let mut push_link = |a: &ApNode, b: &ApNode, horizontal: bool, gap: usize| {
            links.push(Link {
                id: 0, // fixed up below
                a: a.id,
                b: b.id,
                mid_x: 0.5 * (a.x_ft + b.x_ft),
                mid_y: 0.5 * (a.y_ft + b.y_ft),
                horizontal,
                gap,
            });
        };
        for r in 0..rows {
            for c in 0..cols {
                let cur = &nodes[node_id(c, r)];
                if c + 1 < cols {
                    push_link(cur, &nodes[node_id(c + 1, r)], true, c);
                }
                if r + 1 < rows {
                    push_link(cur, &nodes[node_id(c, r + 1)], false, 0);
                }
            }
        }
        for (i, l) in links.iter_mut().enumerate() {
            l.id = i;
        }

        // Sensing lines: one per column-gap, gathering that gap's horizontal links.
        let mut lines = Vec::new();
        for g in 0..cols.saturating_sub(1) {
            let link_ids: Vec<usize> = links
                .iter()
                .filter(|l| l.horizontal && l.gap == g)
                .map(|l| l.id)
                .collect();
            lines.push(SensingLine {
                id: g,
                gap: g,
                x_ft: (g as f32 + 0.5) * spacing_ft,
                link_ids,
            });
        }

        Self {
            spacing_ft,
            cols,
            rows,
            nodes,
            links,
            lines,
            zones: Vec::new(),
            gates: Vec::new(),
        }
    }

    /// Reference shop floor: a 7-column × 3-row AP grid at 50 ft spacing
    /// (300 ft × 100 ft), split into four operational zones connected in a line
    /// by three gates. This is the layout the dashboard and simulator default to.
    pub fn default_shopfloor() -> Self {
        let mut mesh = Self::grid(7, 3, 50.0);
        mesh.zones = vec![
            Zone { id: 0, name: "Receiving".into(), col_lo: 0, col_hi: 1 },
            Zone { id: 1, name: "Storage-A".into(), col_lo: 2, col_hi: 3 },
            Zone { id: 2, name: "Storage-B".into(), col_lo: 4, col_hi: 5 },
            Zone { id: 3, name: "Shipping".into(), col_lo: 6, col_hi: 6 },
        ];
        // Each gate is a non-overlapping adjacent line pair (line_lo, line_lo+1)
        // so every sensing line belongs to exactly one gate. 6 lines (L0..L5)
        // → three gates at line_lo = 0, 2, 4, one per zone boundary.
        mesh.add_gate("Dock→Storage", 0, 0, 1);
        mesh.add_gate("Storage A→B", 2, 1, 2);
        mesh.add_gate("Storage→Ship", 4, 2, 3);
        mesh
    }

    /// Add a gate whose low tripwire line is `line_lo` and high line is
    /// `line_lo + 1`, moving flow between `zone_lo` and `zone_hi`.
    pub fn add_gate(&mut self, name: &str, line_lo: usize, zone_lo: usize, zone_hi: usize) {
        let line_hi = line_lo + 1;
        let x_ft = 0.5 * (self.lines[line_lo].x_ft + self.lines[line_hi].x_ft);
        let id = self.gates.len();
        self.gates.push(Gate {
            id,
            name: name.into(),
            line_lo,
            line_hi,
            zone_lo,
            zone_hi,
            x_ft,
        });
    }

    pub fn floor_len_ft(&self) -> f32 {
        (self.cols.saturating_sub(1)) as f32 * self.spacing_ft
    }
    pub fn floor_width_ft(&self) -> f32 {
        (self.rows.saturating_sub(1)) as f32 * self.spacing_ft
    }

    /// Which zone contains AP-column `col` (falls back to nearest by range).
    pub fn zone_of_col(&self, col: usize) -> Option<usize> {
        self.zones
            .iter()
            .find(|z| col >= z.col_lo && col <= z.col_hi)
            .map(|z| z.id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_has_expected_nodes_and_lines() {
        let m = WarehouseMesh::grid(7, 3, 50.0);
        assert_eq!(m.nodes.len(), 21);
        // 6 sensing lines (column-gaps) for 7 columns.
        assert_eq!(m.lines.len(), 6);
        // Each line has one horizontal link per row.
        for l in &m.lines {
            assert_eq!(l.link_ids.len(), 3);
        }
    }

    #[test]
    fn default_shopfloor_is_wired_consistently() {
        let m = WarehouseMesh::default_shopfloor();
        assert_eq!(m.zones.len(), 4);
        assert_eq!(m.gates.len(), 3);
        assert_eq!(m.floor_len_ft(), 300.0);
        for g in &m.gates {
            assert_eq!(g.line_hi, g.line_lo + 1);
            // Gate x sits strictly between its two tripwire lines.
            assert!(m.lines[g.line_lo].x_ft < g.x_ft && g.x_ft < m.lines[g.line_hi].x_ft);
            // Gate connects two real, distinct zones.
            assert!(g.zone_lo < m.zones.len() && g.zone_hi < m.zones.len());
            assert_ne!(g.zone_lo, g.zone_hi);
        }
    }

    #[test]
    fn zone_lookup_covers_every_column() {
        let m = WarehouseMesh::default_shopfloor();
        for c in 0..m.cols {
            assert!(m.zone_of_col(c).is_some(), "column {c} has no zone");
        }
    }
}
