//! Axum server — serves the self-contained live dashboard and a JSON snapshot
//! of the current flow state that the dashboard polls.
//!
//! Routes:
//! * `GET /`               → the dashboard (single embedded HTML file)
//! * `GET /api/flow/state` → the latest [`snapshot_json`] the runtime published
//! * `GET /healthz`        → `"ok"`

use crate::flow::FlowEngine;
use axum::{
    extract::State,
    response::{Html, IntoResponse, Json},
    routing::get,
    Router,
};
use serde_json::{json, Value};
use std::sync::Arc;
use tokio::sync::Mutex;

/// Shared latest-snapshot cell the runtime writes and the server reads.
pub type SharedState = Arc<Mutex<Value>>;

const DASHBOARD_HTML: &str = include_str!("../assets/dashboard.html");

pub fn new_shared() -> SharedState {
    Arc::new(Mutex::new(Value::Null))
}

/// Build the JSON the dashboard renders: static geometry + live energies,
/// running stats, mover positions, and the recent-crossings feed.
pub fn snapshot_json(
    eng: &FlowEngine,
    link_energy: &[f32],
    movers: &[(f32, f32, &'static str)],
    tick: u64,
    throughput_per_min: f32,
    synthetic: bool,
) -> Value {
    let mesh = eng.mesh();

    let nodes: Vec<Value> = mesh
        .nodes
        .iter()
        .map(|n| json!({ "id": n.id, "x": n.x_ft, "y": n.y_ft }))
        .collect();

    let links: Vec<Value> = mesh
        .links
        .iter()
        .map(|l| {
            json!({
                "a": l.a,
                "b": l.b,
                "horizontal": l.horizontal,
                "energy": link_energy.get(l.id).copied().unwrap_or(0.0),
            })
        })
        .collect();

    let lines: Vec<Value> = mesh
        .lines
        .iter()
        .map(|l| {
            json!({
                "id": l.id,
                "x": l.x_ft,
                "energy": eng.line_energy().get(l.id).copied().unwrap_or(0.0),
            })
        })
        .collect();

    let gates: Vec<Value> = mesh
        .gates
        .iter()
        .map(|g| {
            json!({
                "id": g.id,
                "name": g.name,
                "x": g.x_ft,
                "forward": eng.gate_forward[g.id],
                "reverse": eng.gate_reverse[g.id],
            })
        })
        .collect();

    let zones: Vec<Value> = mesh
        .zones
        .iter()
        .map(|z| {
            let x_lo = z.col_lo as f32 * mesh.spacing_ft;
            let x_hi = z.col_hi as f32 * mesh.spacing_ft;
            json!({
                "id": z.id,
                "name": z.name,
                "x_lo": x_lo,
                "x_hi": x_hi,
                "net": eng.occupancy[z.id],
            })
        })
        .collect();

    let movers: Vec<Value> = movers
        .iter()
        .map(|(x, y, size)| json!({ "x": x, "y": y, "size": size }))
        .collect();

    let recent: Vec<Value> = eng
        .recent()
        .rev()
        .take(12)
        .map(|c| {
            json!({
                "tick": c.tick,
                "gate": c.gate_name,
                "direction": c.direction.as_str(),
                "from": c.from_zone_name,
                "to": c.to_zone_name,
                "size": c.size.as_str(),
                "size_confidence": c.size_confidence,
            })
        })
        .collect();

    json!({
        "tick": tick,
        "synthetic": synthetic,
        "data_gated": true,
        "floor": { "len_ft": mesh.floor_len_ft(), "width_ft": mesh.floor_width_ft(), "spacing_ft": mesh.spacing_ft },
        "kpis": {
            "total_crossings": eng.total_crossings,
            "throughput_per_min": throughput_per_min,
            "size_counts": {
                "tote": eng.size_counts[0],
                "carton": eng.size_counts[1],
                "pallet": eng.size_counts[2],
                "forklift": eng.size_counts[3],
            },
        },
        "mesh": { "nodes": nodes, "links": links, "lines": lines, "gates": gates, "zones": zones },
        "movers": movers,
        "recent": recent,
    })
}

pub fn router(state: SharedState) -> Router {
    Router::new()
        .route("/", get(dashboard))
        .route("/api/flow/state", get(state_handler))
        .route("/healthz", get(|| async { "ok" }))
        .with_state(state)
}

async fn dashboard() -> impl IntoResponse {
    Html(DASHBOARD_HTML)
}

async fn state_handler(State(state): State<SharedState>) -> impl IntoResponse {
    let v = state.lock().await.clone();
    Json(v)
}

/// Bind and serve until the process is stopped.
pub async fn serve(addr: std::net::SocketAddr, state: SharedState) -> Result<(), Box<dyn std::error::Error>> {
    let listener = tokio::net::TcpListener::bind(addr).await?;
    tracing::info!(%addr, "package-flow dashboard listening");
    axum::serve(listener, router(state)).await?;
    Ok(())
}
