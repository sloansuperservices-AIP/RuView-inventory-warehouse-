//! `cog-package-flow` — WiFi-CSI package-flow tracker binary (ADR-266).
//!
//! Subcommands:
//!   cog-package-flow version
//!   cog-package-flow manifest          # cog descriptor (JSON)
//!   cog-package-flow health            # self-check, emits health.ok
//!   cog-package-flow simulate [--ticks N] [--tick-ms MS]   # headless JSONL
//!   cog-package-flow serve [--addr IP:PORT]                # live dashboard
//!   cog-package-flow run --sensing-url URL                 # real CSI (calibrate first)

use clap::{Parser, Subcommand};
use cog_package_flow::{
    geometry::WarehouseMesh,
    publisher,
    runtime::{self, RunParams},
    server,
    simulator::SimConfig,
    COG_ID, COG_VERSION,
};

#[derive(Parser)]
#[command(name = "cog-package-flow", version = COG_VERSION)]
struct Cli {
    #[command(subcommand)]
    command: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Print `<cog-id> <version>`.
    Version,
    /// Print the cog descriptor as JSON.
    Manifest,
    /// Run a self-check and emit a `health.ok` event.
    Health,
    /// Run the deterministic synthetic warehouse, emitting JSONL flow events.
    Simulate {
        #[arg(long, default_value_t = 3000)]
        ticks: u64,
        #[arg(long, default_value_t = 50)]
        tick_ms: u64,
        #[arg(long, default_value_t = 0xC0FFEE)]
        seed: u64,
    },
    /// Serve the live dashboard (drives the simulator behind it).
    Serve {
        #[arg(long, default_value = "127.0.0.1:8787")]
        addr: String,
        #[arg(long, default_value_t = 50)]
        tick_ms: u64,
        #[arg(long, default_value_t = 0xC0FFEE)]
        seed: u64,
        /// Also stream JSONL flow events to stdout.
        #[arg(long)]
        emit: bool,
    },
    /// Drive the flow engine from a live sensing-server (real CSI).
    Run {
        #[arg(long, default_value = "http://127.0.0.1:3000/api/v1/sensing/latest")]
        sensing_url: String,
        #[arg(long, default_value_t = 50)]
        tick_ms: u64,
    },
}

fn main() -> std::process::ExitCode {
    init_logging();
    let cli = Cli::parse();
    let result = match cli.command {
        Cmd::Version => cmd_version(),
        Cmd::Manifest => cmd_manifest(),
        Cmd::Health => cmd_health(),
        Cmd::Simulate { ticks, tick_ms, seed } => cmd_simulate(ticks, tick_ms, seed),
        Cmd::Serve { addr, tick_ms, seed, emit } => cmd_serve(addr, tick_ms, seed, emit),
        Cmd::Run { sensing_url, tick_ms } => cmd_run(sensing_url, tick_ms),
    };
    match result {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("cog-package-flow: {err}");
            std::process::ExitCode::FAILURE
        }
    }
}

fn init_logging() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_target(false)
        .try_init();
}

fn cmd_version() -> Result<(), Box<dyn std::error::Error>> {
    println!("{COG_ID} {COG_VERSION}");
    Ok(())
}

fn cmd_manifest() -> Result<(), Box<dyn std::error::Error>> {
    let mesh = WarehouseMesh::default_shopfloor();
    let spec = serde_json::json!({
        "id": COG_ID,
        "version": COG_VERSION,
        "adr": "ADR-266",
        "summary": "WiFi-CSI package-flow tracker — directional RF tripwires over an existing AP grid.",
        "inputs": ["wifi-csi-motion-energy (per bistatic link)"],
        "outputs": ["package.crossing", "flow.summary"],
        // Honest disclosure — see lib.rs.
        "counts": "flow events (gate crossings), not static inventory",
        "synthetic_default": true,
        "data_gated": true,
        "default_mesh": {
            "ap_grid": { "cols": mesh.cols, "rows": mesh.rows, "spacing_ft": mesh.spacing_ft },
            "zones": mesh.zones.iter().map(|z| &z.name).collect::<Vec<_>>(),
            "gates": mesh.gates.iter().map(|g| &g.name).collect::<Vec<_>>(),
        },
    });
    println!("{}", serde_json::to_string_pretty(&spec)?);
    Ok(())
}

fn cmd_health() -> Result<(), Box<dyn std::error::Error>> {
    // Build the default mesh + engine and push a few synthetic ticks through to
    // prove the pipeline is wired end-to-end.
    use cog_package_flow::flow::FlowEngine;
    use cog_package_flow::simulator::Simulator;
    let mesh = WarehouseMesh::default_shopfloor();
    let mut sim = Simulator::new(mesh.clone(), SimConfig::default());
    let mut eng = FlowEngine::with_defaults(mesh);
    for t in 0..50 {
        let e = sim.step(t);
        let _ = eng.ingest_links(t, &e);
    }
    publisher::health_ok(COG_ID, true);
    Ok(())
}

fn cmd_simulate(ticks: u64, tick_ms: u64, seed: u64) -> Result<(), Box<dyn std::error::Error>> {
    let mesh = WarehouseMesh::default_shopfloor();
    let sim_cfg = SimConfig { seed, ..Default::default() };
    publisher::run_started(COG_ID, "simulate", true, tick_ms, (mesh.cols, mesh.rows, mesh.spacing_ft));
    let params = RunParams { tick_ms, max_ticks: Some(ticks), emit_jsonl: true, summary_every: 200 };
    tokio_rt()?.block_on(runtime::run_simulator(mesh, sim_cfg, params, None))
}

fn cmd_serve(addr: String, tick_ms: u64, seed: u64, emit: bool) -> Result<(), Box<dyn std::error::Error>> {
    let mesh = WarehouseMesh::default_shopfloor();
    let sim_cfg = SimConfig { seed, ..Default::default() };
    let sock: std::net::SocketAddr = addr.parse().map_err(|e| format!("bad --addr {addr}: {e}"))?;
    publisher::run_started(COG_ID, "serve", true, tick_ms, (mesh.cols, mesh.rows, mesh.spacing_ft));
    eprintln!("cog-package-flow: dashboard on http://{sock}/");

    let rt = tokio_rt()?;
    rt.block_on(async move {
        let shared = server::new_shared();
        let srv_state = shared.clone();
        tokio::spawn(async move {
            if let Err(e) = server::serve(sock, srv_state).await {
                tracing::error!(error = %e, "dashboard server exited");
            }
        });
        let params = RunParams { tick_ms, max_ticks: None, emit_jsonl: emit, summary_every: 200 };
        runtime::run_simulator(mesh, sim_cfg, params, Some(shared)).await
    })
}

fn cmd_run(sensing_url: String, tick_ms: u64) -> Result<(), Box<dyn std::error::Error>> {
    let mesh = WarehouseMesh::default_shopfloor();
    publisher::run_started(COG_ID, "run", false, tick_ms, (mesh.cols, mesh.rows, mesh.spacing_ft));
    let params = RunParams { tick_ms, max_ticks: None, emit_jsonl: true, summary_every: 200 };
    tokio_rt()?.block_on(runtime::run_real(mesh, sensing_url, params, None))
}

fn tokio_rt() -> Result<tokio::runtime::Runtime, Box<dyn std::error::Error>> {
    Ok(tokio::runtime::Builder::new_multi_thread().enable_all().build()?)
}
