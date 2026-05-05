#!/usr/bin/env python3
"""Run per-coordinate ns-3/Sionna TCP experiments and generate real plots."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import getpass
import json
import math
import os
import signal
import shlex
import socket
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

CNAM_ROUTE_POINTS = [
    (32.0, -9.0, 1.5),
    (-72.0, -31.0, 1.5),
    (-120.0, 90.0, 1.5),
    (-208.0, -101.0, 1.5),
    (205.0, 177.0, 1.5),
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output-root", type=Path, default=Path("results"))
    p.add_argument("--transport", default="tcp", choices=["tcp", "udp"])
    p.add_argument("--tcpVariant", default="TcpNewReno")
    p.add_argument("--packetSize", type=int, default=512)
    p.add_argument("--maxBytes", type=int, default=0)
    p.add_argument("--appStart", type=float, default=1.0)
    p.add_argument("--appStop", type=float, default=10.0)
    p.add_argument("--simTime", type=float, default=10.0)
    p.add_argument("--port", type=int, default=9000)
    p.add_argument("--enable-tap", action="store_true", help="Enable ns-3 TapBridge so Linux tcpdump can observe TAP-side packets")
    p.add_argument("--enable-tcpdump", action="store_true", help="Start tcpdump from this runner for the whole experiment")
    p.add_argument("--tcpdump-interface", default="any")
    p.add_argument("--sumo-cnam-route", action="store_true")
    p.add_argument("--sumo-max-coordinates", type=int)
    p.add_argument("--start-sionna-server", action="store_true")
    p.set_defaults(
        coordinates=None,
        sionna="true",
        shutdownSionna="false",
        ns3_program="sionna-ns3-to-linux-tap",
        tap_name="sionna-tap0",
        tap_mode="ConfigureLocal",
        enable_ns3_pcap=False,
        tcpdump_binary="tcpdump",
        tcpdump_filter=None,
        tcpdump_use_sudo=True,
        tcpdump_output=None,
        radio_map_summary_csv=Path("sionna-radio-maps/radio_map_summary.csv"),
        sumo_config=None,
        sumo_binary="sumo",
        sumo_step_length=1.0,
        sumo_start_time=0.0,
        sumo_end_time=None,
        sumo_vehicle_id=None,
        sumo_z=1.5,
        sumo_skip_steps=0,
        verify_sumo_only=False,
        sionna_server_script=Path("src/sionna/sionna_v1_server_script.py"),
        sionna_port=8103,
        sionna_startup_timeout=180.0,
        route_overlay_radius=2.5,
        route_overlay_stride=1,
        radio_map_samples=50000,
        radio_map_render_samples=128,
        radio_map_resolution=[1920, 1080],
        radio_map_camera_mode="birdseye",
    )
    return p.parse_args()


def new_run_root(output_root: Path) -> Path:
    root = output_root / f"sionna_ns3_experiment_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    root.mkdir(parents=True)
    return root


def read_coordinates(path: Path) -> list[dict]:
    with path.open(newline="") as f:
        rows = []
        for i, row in enumerate(csv.DictReader(f)):
            rows.append({
                "coordinate_index": int(row.get("coordinate_index") or row.get("index") or i),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row.get("z") or 1.5),
                "time_seconds": row.get("time_seconds", ""),
                "vehicle_id": row.get("vehicle_id", ""),
            })
    if not rows:
        raise RuntimeError(f"No coordinates in {path}")
    return rows


def write_rows(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({k for r in rows for k in r})
    preferred = [
        "coordinate_index", "time_seconds", "vehicle_id", "x", "y", "z", "speed", "angle",
        "lane_id", "edge_id", "transport", "tcp_variant", "throughput_mbps",
        "packet_delivery_ratio", "packet_loss_ratio", "average_delay_ms", "average_jitter_ms",
        "rss_dbm", "path_gain_db", "sinr_db", "valid_paths", "retransmissions",
        "average_rtt_ms", "final_cwnd", "average_cwnd",
        "step_index", "distance_along_path_m", "distance_from_tx_m", "path_gain_dB",
        "path_gain_linear", "rss_dBm", "rss_watt", "sinr_dB", "sinr_linear",
        "valid_paths_count", "los_available", "los_present",
        "strongest_path_gain_dB", "strongest_path_delay_ns", "rms_delay_spread_ns",
        "interaction_los_count", "interaction_reflection_count", "interaction_diffraction_count",
        "interaction_refraction_count", "interaction_diffuse_count", "warnings",
    ]
    fields = [f for f in preferred if f in fields] + [f for f in fields if f not in preferred]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def count_csv_rows(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open(newline="") as f:
        return max(sum(1 for _ in csv.DictReader(f)), 0)


def read_csv_rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def generate_cnam_sumo_scenario(run_root: Path) -> Path:
    scenario_dir = run_root / "sumo_cnam_scenario"
    scenario_dir.mkdir(parents=True, exist_ok=True)
    nodes_path = scenario_dir / "cnam.nod.xml"
    edges_path = scenario_dir / "cnam.edg.xml"
    net_path = scenario_dir / "cnam.net.xml"
    route_path = scenario_dir / "cnam.rou.xml"
    config_path = scenario_dir / "cnam.sumocfg"

    node_lines = ["<nodes>"]
    for index, (x, y, _z) in enumerate(CNAM_ROUTE_POINTS):
        node_lines.append(f'  <node id="n{index}" x="{x}" y="{y}" type="priority"/>')
    node_lines.append("</nodes>\n")
    write_text(nodes_path, "\n".join(node_lines))

    edge_lines = ["<edges>"]
    for index in range(len(CNAM_ROUTE_POINTS) - 1):
        edge_lines.append(
            f'  <edge id="e{index}" from="n{index}" to="n{index + 1}" priority="1" '
            'numLanes="1" speed="13.9"/>'
        )
    edge_lines.append("</edges>\n")
    write_text(edges_path, "\n".join(edge_lines))

    netconvert = shutil.which("netconvert")
    if not netconvert:
        raise RuntimeError("netconvert was not found in PATH; install SUMO tools to generate CNAM SUMO route")
    command = [
        netconvert,
        "--node-files",
        str(nodes_path),
        "--edge-files",
        str(edges_path),
        "--output-file",
        str(net_path),
        "--offset.disable-normalization",
        "true",
    ]
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"netconvert failed:\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}")

    route_edges = " ".join(f"e{index}" for index in range(len(CNAM_ROUTE_POINTS) - 1))
    write_text(
        route_path,
        "\n".join(
            [
                "<routes>",
                '  <vType id="cnam_car" accel="2.6" decel="4.5" sigma="0.5" length="4.5" maxSpeed="13.9"/>',
                f'  <route id="cnam_route" edges="{route_edges}"/>',
                '  <vehicle id="car_1" type="cnam_car" route="cnam_route" depart="0" departSpeed="max"/>',
                "</routes>\n",
            ]
        ),
    )
    write_text(
        config_path,
        "\n".join(
            [
                "<configuration>",
                "  <input>",
                '    <net-file value="cnam.net.xml"/>',
                '    <route-files value="cnam.rou.xml"/>',
                "  </input>",
                "  <time>",
                '    <begin value="0"/>',
                "  </time>",
                "</configuration>\n",
            ]
        ),
    )
    return config_path


def generate_sumo_coordinates(args: argparse.Namespace, run_root: Path) -> tuple[Path | None, dict]:
    if args.sumo_cnam_route and args.sumo_config is None:
        args.sumo_config = generate_cnam_sumo_scenario(run_root)
        if not args.sumo_vehicle_id:
            args.sumo_vehicle_id = "car_1"

    meta = {
        "sumo_config": str(args.sumo_config) if args.sumo_config else None,
        "sumo_cnam_route": args.sumo_cnam_route,
        "sumo_binary": args.sumo_binary,
        "sumo_binary_available": shutil.which(args.sumo_binary) is not None,
        "traci_available": False,
    }
    if not args.sumo_config:
        meta["status"] = "not_requested"
        return None, meta
    if not args.sumo_config.exists():
        meta["status"] = "missing_sumo_config"
        meta["message"] = f"SUMO config file not found: {args.sumo_config}"
        return None, meta
    if not meta["sumo_binary_available"]:
        meta["status"] = "missing_sumo_binary"
        meta["message"] = f"'{args.sumo_binary}' not found in PATH"
        return None, meta
    try:
        import traci  # type: ignore
    except ImportError:
        meta["status"] = "missing_traci"
        meta["message"] = "Python module 'traci' is not installed"
        return None, meta

    meta["traci_available"] = True
    port = 8873
    cmd = [
        args.sumo_binary, "-c", str(args.sumo_config), "--remote-port", str(port),
        "--step-length", str(args.sumo_step_length), "--begin", str(args.sumo_start_time),
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    traci_connected = False
    for _ in range(40):
        if proc.poll() is not None:
            stdout, stderr = proc.communicate(timeout=5)
            meta.update({
                "status": "sumo_exited_before_traci_connect",
                "sumo_returncode": proc.returncode,
                "sumo_stdout": stdout[-4000:],
                "sumo_stderr": stderr[-4000:],
            })
            return None, meta
        try:
            traci.connect(port=port, numRetries=1, label="default")
            traci.switch("default")
            traci_connected = True
            break
        except Exception:
            time.sleep(0.25)
    else:
        stdout, stderr = proc.communicate(timeout=5)
        meta.update({"status": "traci_connect_failed", "sumo_stdout": stdout[-4000:], "sumo_stderr": stderr[-4000:]})
        return None, meta

    rows = []
    try:
        step = 0
        idx = 0
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            now = float(traci.simulation.getTime())
            if args.sumo_end_time is not None and now > args.sumo_end_time:
                break
            if step % (args.sumo_skip_steps + 1) == 0:
                ids = [args.sumo_vehicle_id] if args.sumo_vehicle_id else list(traci.vehicle.getIDList())
                for vid in ids:
                    if vid not in traci.vehicle.getIDList():
                        continue
                    x, y = traci.vehicle.getPosition(vid)
                    rows.append({
                        "coordinate_index": idx, "time_seconds": now, "vehicle_id": vid,
                        "x": x, "y": y, "z": args.sumo_z,
                        "speed": traci.vehicle.getSpeed(vid),
                        "angle": traci.vehicle.getAngle(vid),
                        "lane_id": traci.vehicle.getLaneID(vid),
                        "edge_id": traci.vehicle.getRoadID(vid),
                    })
                    idx += 1
                    if args.sumo_max_coordinates and idx >= args.sumo_max_coordinates:
                        break
            if args.sumo_max_coordinates and idx >= args.sumo_max_coordinates:
                break
            step += 1
    finally:
        if traci_connected:
            try:
                traci.close(False)
            except Exception:
                pass
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
        stdout, stderr = proc.communicate(timeout=10)
        meta["sumo_stdout"] = stdout[-4000:]
        meta["sumo_stderr"] = stderr[-4000:]

    if not rows:
        meta["status"] = "no_coordinates"
        return None, meta
    path = run_root / "sumo_coordinates.csv"
    write_rows(path, rows)
    meta.update({"status": "ok", "coordinate_count": len(rows), "output_csv": str(path)})
    return path, meta


def seconds(value: str) -> float:
    value = (value or "0").strip().lstrip("+")
    for suffix, scale in (("ns", 1e-9), ("us", 1e-6), ("ms", 1e-3), ("s", 1.0)):
        if value.endswith(suffix):
            return float(value[:-len(suffix)]) * scale
    return float(value or 0)


def parse_flowmon(path: Path, port: int, app_start: float, app_stop: float) -> dict:
    out = {"txPackets": 0, "rxPackets": 0, "lostPackets": 0, "txBytes": 0, "rxBytes": 0,
           "throughput_mbps": 0.0, "packet_delivery_ratio": 0.0, "packet_loss_ratio": 0.0,
           "average_delay_ms": 0.0, "average_jitter_ms": 0.0}
    if not path.exists():
        return out
    root = ET.parse(path).getroot()
    classifiers = {e.attrib.get("flowId"): e for e in root.findall(".//Ipv4FlowClassifier/Flow")}
    flows = root.findall(".//FlowStats/Flow")
    matches = [f for f in flows if classifiers.get(f.attrib.get("flowId")) is not None
               and classifiers[f.attrib.get("flowId")].attrib.get("destinationPort") == str(port)]
    if not matches and flows:
        matches = flows
    if not matches:
        return out
    f = max(matches, key=lambda e: int(e.attrib.get("rxBytes", "0")))
    txp, rxp = int(f.attrib.get("txPackets", 0)), int(f.attrib.get("rxPackets", 0))
    lost = int(f.attrib.get("lostPackets", 0))
    txb, rxb = int(f.attrib.get("txBytes", 0)), int(f.attrib.get("rxBytes", 0))
    first = seconds(f.attrib.get("timeFirstTxPacket", f"{app_start}s"))
    last = seconds(f.attrib.get("timeLastRxPacket", f"{app_stop}s"))
    duration = max(last - first, app_stop - app_start, 1e-9)
    delay_sum = seconds(f.attrib.get("delaySum", "0s"))
    jitter_sum = seconds(f.attrib.get("jitterSum", "0s"))
    out.update({
        "txPackets": txp, "rxPackets": rxp, "lostPackets": lost, "txBytes": txb, "rxBytes": rxb,
        "throughput_mbps": rxb * 8 / duration / 1e6,
        "packet_delivery_ratio": rxp / txp if txp else 0.0,
        "packet_loss_ratio": lost / txp if txp else 0.0,
        "average_delay_ms": delay_sum / rxp * 1000 if rxp else 0.0,
        "average_jitter_ms": jitter_sum / max(rxp - 1, 1) * 1000 if rxp > 1 else 0.0,
    })
    return out


def float_or_blank(v: object) -> float | str:
    try:
        if v in ("", None):
            return ""
        x = float(v)
        return x if math.isfinite(x) else ""
    except Exception:
        return ""


def parse_tcp_trace(path: Path, value_column: str) -> list[float]:
    vals = []
    for row in read_csv_rows(path):
        val = float_or_blank(row.get(value_column))
        if val != "":
            vals.append(float(val))
    return vals


def find_sionna_row_for_coord(rows: list[dict], coord: dict, tolerance_m: float = 1e-3) -> dict:
    matches = []
    for row in rows:
        try:
            distance = math.dist(
                (float(row.get("rx_x", "nan")), float(row.get("rx_y", "nan")), float(row.get("rx_z", "nan"))),
                (float(coord["x"]), float(coord["y"]), float(coord["z"])),
            )
        except (TypeError, ValueError):
            continue
        if distance <= tolerance_m:
            matches.append(row)
    return matches[-1] if matches else {}


def plot_xy(path: Path, x: list[float], y: list[float], title: str, ylabel: str, xlabel: str = "coordinate") -> bool:
    points = [(a, b) for a, b in zip(x, y) if b != "" and math.isfinite(float(b))]
    if len(points) < 2:
        return False
    unique_values = {round(float(b), 12) for _a, b in points}
    if len(unique_values) < 2:
        return False
    import matplotlib.pyplot as plt  # type: ignore
    xs, ys = zip(*points)
    plt.figure(figsize=(7, 4))
    plt.plot(xs, ys, marker="o")
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(path)
    plt.close()
    return True


def numeric_or_none(value: object) -> float | None:
    value = float_or_blank(value)
    return None if value == "" else float(value)


def make_trajectory_metrics(coords: list[dict], summaries: list[dict], args: argparse.Namespace, sumo_meta: dict) -> tuple[list[dict], dict]:
    rows = []
    warnings = set()
    distance_along = 0.0
    previous = None
    for index, (coord, summary) in enumerate(zip(coords, summaries)):
        current = (float(coord["x"]), float(coord["y"]), float(coord["z"]))
        if previous is not None:
            distance_along += math.dist(previous, current)
        previous = current

        path_gain = numeric_or_none(summary.get("path_gain_db"))
        rss = numeric_or_none(summary.get("rss_dbm"))
        sinr = numeric_or_none(summary.get("sinr_db"))
        distance_from_tx = numeric_or_none(summary.get("distance_from_tx_m"))
        los_present = summary.get("los_present", "")
        valid_paths_count = summary.get("valid_paths_count", "")

        row_warnings = []
        if not summary.get("sionna_recomputed_for_step"):
            row_warnings.append("No Sionna summary row was produced for this sampled RX position")
        for metric_name, metric_value in [
            ("path_gain_dB", path_gain),
            ("rss_dBm", rss),
            ("sinr_dB", sinr),
            ("distance_from_tx_m", distance_from_tx),
        ]:
            if metric_value is None:
                row_warnings.append(f"{metric_name} unavailable")
        path_gain_linear = None if path_gain is None else 10 ** (path_gain / 10.0)
        rss_watt = None if rss is None else 10 ** ((rss - 30.0) / 10.0)
        sinr_linear = None if sinr is None else 10 ** (sinr / 10.0)

        unavailable = [
            "path_delays_ns",
            "path_coefficients",
            "interaction type counts",
            "power delay profile",
            "channel frequency response",
        ]
        row_warnings.extend(f"{name} unavailable from current exported Sionna summary" for name in unavailable)
        warnings.update(row_warnings)

        rows.append({
            "step_index": index,
            "time_seconds": coord.get("time_seconds", ""),
            "vehicle_id": coord.get("vehicle_id", ""),
            "x": coord["x"],
            "y": coord["y"],
            "z": coord["z"],
            "distance_along_path_m": distance_along,
            "distance_from_tx_m": "" if distance_from_tx is None else distance_from_tx,
            "path_gain_dB": "" if path_gain is None else path_gain,
            "path_gain_linear": "" if path_gain_linear is None else path_gain_linear,
            "rss_dBm": "" if rss is None else rss,
            "rss_watt": "" if rss_watt is None else rss_watt,
            "sinr_dB": "" if sinr is None else sinr,
            "sinr_linear": "" if sinr_linear is None else sinr_linear,
            "valid_paths_count": valid_paths_count,
            "los_available": summary.get("los_available", ""),
            "los_present": los_present,
            "strongest_path_gain_dB": "" if path_gain is None else path_gain,
            "strongest_path_delay_ns": summary.get("strongest_path_delay_ns", ""),
            "rms_delay_spread_ns": "",
            "interaction_los_count": 1 if los_present is True else "",
            "interaction_reflection_count": "",
            "interaction_diffraction_count": "",
            "interaction_refraction_count": "",
            "interaction_diffuse_count": "",
            "warnings": "; ".join(row_warnings),
        })

    metadata = {
        "sumo_used": bool(args.sumo_config or args.sumo_cnam_route),
        "sumo_config": str(args.sumo_config) if args.sumo_config else None,
        "vehicle_id": args.sumo_vehicle_id,
        "number_of_raw_sumo_points": sumo_meta.get("coordinate_count", 0),
        "number_of_sampled_rx_points": len(rows),
        "sumo_step_length": args.sumo_step_length,
        "sumo_skip_steps": args.sumo_skip_steps,
        "sumo_max_coordinates": args.sumo_max_coordinates,
        "sionna_recomputed_per_sampled_position": bool(rows) and all(bool(s.get("sionna_recomputed_for_step")) for s in summaries),
        "sionna_evaluation_mode": "loop_per_sample_via_ns3_position_update_and_on_request_radio_map_export",
        "los_detection_available": any(row["los_available"] is True for row in rows),
        "ns3_run_per_step": True,
        "ns3_flowmonitor_metrics_are_per_step": True,
        "warnings": sorted(warnings),
    }
    return rows, metadata


def plot_trajectory_outputs(plot_dir: Path, rows: list[dict]) -> dict:
    skipped = {}
    if not rows:
        return {"trajectory_plots": "No trajectory rows available"}
    steps = [float(row["step_index"]) for row in rows]
    distance = [float(row["distance_along_path_m"]) for row in rows]
    for key, filename, ylabel in [
        ("rss_dBm", "rss_vs_step.png", "dBm"),
        ("path_gain_dB", "path_gain_vs_step.png", "dB"),
        ("sinr_dB", "sinr_vs_step.png", "dB"),
        ("valid_paths_count", "valid_paths_vs_step.png", "count"),
        ("los_present", "los_vs_step.png", "LOS present"),
        ("rms_delay_spread_ns", "delay_spread_vs_step.png", "ns"),
    ]:
        if not plot_xy(plot_dir / filename, steps, [row.get(key, "") for row in rows], key, ylabel, "step"):
            skipped[filename] = "No varying finite samples available"
    for key, filename, ylabel in [
        ("rss_dBm", "rss_vs_distance_along_path.png", "dBm"),
        ("sinr_dB", "sinr_vs_distance_along_path.png", "dB"),
        ("path_gain_dB", "path_gain_vs_distance_along_path.png", "dB"),
    ]:
        if not plot_xy(plot_dir / filename, distance, [row.get(key, "") for row in rows], key, ylabel, "distance along path [m]"):
            skipped[filename] = "No varying finite samples available"

    points = [(float(row["x"]), float(row["y"]), numeric_or_none(row.get("rss_dBm"))) for row in rows]
    real_points = [(x, y, v) for x, y, v in points if v is not None]
    if not real_points:
        skipped["trajectory_signal_map.png"] = "No finite RSS samples available"
        return skipped
    import matplotlib.pyplot as plt  # type: ignore
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    color_x = [p[0] for p in real_points]
    color_y = [p[1] for p in real_points]
    color_v = [p[2] for p in real_points]
    plt.figure(figsize=(9, 7))
    plt.plot(xs, ys, color="0.35", linewidth=1.4, alpha=0.8, label="SUMO sampled RX path")
    scatter = plt.scatter(color_x, color_y, c=color_v, cmap="plasma", s=85, edgecolors="black", linewidths=0.5, label="RX samples by RSS")
    plt.scatter([xs[-1]], [ys[-1]], marker="*", c="cyan", edgecolors="black", s=220, label="final RX sample")
    plt.colorbar(scatter, label="RSS (dBm)")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory signal map")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(plot_dir / "trajectory_signal_map.png", dpi=180)
    plt.close()
    return skipped


def udp_port_bound(port: int) -> bool:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", port))
        return False
    except OSError:
        return True
    finally:
        probe.close()


def wait_for_sionna_server(process: subprocess.Popen, port: int, timeout: float, stdout_path: Path) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Sionna server exited during startup with code {process.returncode}")
        if stdout_path.exists() and "Setup complete." in stdout_path.read_text(encoding="utf-8", errors="ignore"):
            return
        if udp_port_bound(port):
            time.sleep(1.0)
            return
        time.sleep(0.5)
    raise RuntimeError(f"Sionna server did not bind UDP port {port} within {timeout:.0f}s")


def start_sionna_server(args: argparse.Namespace, run_root: Path, route_overlay_csv: Path | None) -> subprocess.Popen:
    log_dir = run_root / "sionna_server"
    log_dir.mkdir(parents=True, exist_ok=True)
    stdout_path = log_dir / "stdout.log"
    stderr_path = log_dir / "stderr.log"
    stdout = stdout_path.open("w", encoding="utf-8")
    stderr = stderr_path.open("w", encoding="utf-8")
    server_script = args.sionna_server_script.resolve()
    radio_map_summary_csv = args.radio_map_summary_csv.resolve()
    radio_map_summary_csv.parent.mkdir(parents=True, exist_ok=True)

    command = [
        sys.executable,
        str(server_script.name),
        "--local-machine",
        "--port",
        str(args.sionna_port),
        "--frequency=2.1e9",
        "--path-to-xml-scenario=scenarios/cnam_scene_new/cnam_scene.xml",
        "--export-radio-map",
        "--radio-map-export-mode=on-request",
        f"--radio-map-dir={radio_map_summary_csv.parent}",
        f"--radio-map-summary-csv={radio_map_summary_csv}",
        "--radio-map-tx-car-id=2",
        "--radio-map-rx-car-id=1",
        f"--radio-map-camera-mode={args.radio_map_camera_mode}",
        "--radio-map-resolution",
        str(args.radio_map_resolution[0]),
        str(args.radio_map_resolution[1]),
        f"--radio-map-samples={args.radio_map_samples}",
        f"--radio-map-render-samples={args.radio_map_render_samples}",
        f"--route-overlay-radius={args.route_overlay_radius}",
        f"--route-overlay-stride={args.route_overlay_stride}",
        "--route-overlay-vehicle-id=car_1",
    ]
    if route_overlay_csv:
        command.append(f"--route-overlay-csv={route_overlay_csv.resolve()}")

    process = subprocess.Popen(
        command,
        cwd=server_script.parent,
        stdout=stdout,
        stderr=stderr,
        text=True,
        env=os.environ | {"PYTHONUNBUFFERED": "1"},
    )
    (log_dir / "command.json").write_text(json.dumps(command, indent=2), encoding="utf-8")
    wait_for_sionna_server(process, args.sionna_port, args.sionna_startup_timeout, stdout_path)
    return process


def stop_sionna_server(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=10)


def start_tcpdump(args: argparse.Namespace, run_root: Path, sudo_password: str | None = None) -> tuple[subprocess.Popen | None, dict]:
    meta = {
        "enabled": args.enable_tcpdump,
        "status": "disabled",
        "pcap": "",
        "stdout_log": "",
        "stderr_log": "",
        "interface": args.tcpdump_interface,
        "filter": "",
        "command": [],
        "warnings": [],
    }
    if not args.enable_tcpdump:
        return None, meta

    if shutil.which(args.tcpdump_binary) is None:
        meta.update({"status": "missing_tcpdump"})
        meta["warnings"].append(f"{args.tcpdump_binary!r} was not found in PATH")
        return None, meta

    raw_root = run_root / "raw"
    raw_root.mkdir(exist_ok=True)
    pcap_path = (args.tcpdump_output or (raw_root / "tcpdump_route.pcap")).resolve()
    stdout_path = raw_root / "tcpdump_stdout.log"
    stderr_path = raw_root / "tcpdump_stderr.log"
    capture_filter = args.tcpdump_filter or f"{args.transport} port {args.port}"
    command = [
        args.tcpdump_binary,
        "-i",
        args.tcpdump_interface,
        "-n",
        "-s",
        "0",
        "-w",
        str(pcap_path),
    ]
    if capture_filter:
        command.extend(shlex.split(capture_filter))
    stdin = None
    if args.tcpdump_use_sudo:
        if sudo_password:
            command = ["sudo", "-S"] + command
            stdin = subprocess.PIPE
        else:
            command = ["sudo", "-n"] + command

    stdout = stdout_path.open("w", encoding="utf-8")
    stderr = stderr_path.open("w", encoding="utf-8")
    env = os.environ.copy()
    if sudo_password:
        env["SUDO_PASSWORD"] = sudo_password
    try:
        process = subprocess.Popen(
            command,
            stdin=stdin,
            stdout=stdout,
            stderr=stderr,
            text=True,
            preexec_fn=os.setsid,
            env=env,
        )
        if sudo_password and process.stdin:
            process.stdin.write(sudo_password + "\n")
            process.stdin.flush()
            process.stdin.close()
    except OSError as exc:
        meta.update({"status": "start_failed"})
        meta["warnings"].append(str(exc))
        stdout.close()
        stderr.close()
        return None, meta

    time.sleep(0.75)
    meta.update({
        "status": "running" if process.poll() is None else "exited_early",
        "pcap": str(pcap_path),
        "stdout_log": str(stdout_path),
        "stderr_log": str(stderr_path),
        "filter": capture_filter,
        "command": command,
    })
    if process.poll() is not None:
        meta["returncode"] = process.returncode
        meta["warnings"].append("tcpdump exited before the experiment loop; check tcpdump_stderr.log")
        stdout.close()
        stderr.close()
        return None, meta
    return process, meta


def stop_tcpdump(process: subprocess.Popen | None, meta: dict) -> dict:
    if process is None:
        return meta
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            meta.setdefault("warnings", []).append("tcpdump did not exit on SIGINT; terminating")
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                meta.setdefault("warnings", []).append("tcpdump did not exit on SIGTERM; killing")
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=5)
    meta["status"] = "completed"
    meta["returncode"] = process.returncode
    pcap_path = Path(meta.get("pcap", ""))
    if pcap_path.exists():
        meta["pcap_size_bytes"] = pcap_path.stat().st_size
        if pcap_path.stat().st_size == 0:
            meta.setdefault("warnings", []).append("tcpdump PCAP is empty")
    return meta


def start_sudo_keepalive(enabled: bool, sudo_password: str | None) -> subprocess.Popen | None:
    if not enabled:
        return None
    command = ["sudo", "-S", "-v"] if sudo_password else ["sudo", "-v"]
    try:
        subprocess.run(
            command,
            input=(sudo_password + "\n") if sudo_password else None,
            text=True,
            check=True,
        )
    except subprocess.CalledProcessError:
        raise RuntimeError("sudo authentication failed; cannot keep sudo alive for TAP/tcpdump capture")
    return subprocess.Popen(
        ["sh", "-c", "while true; do sudo -n -v; sleep 30; done"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )


def stop_sudo_keepalive(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5)
    except Exception:
        try:
            process.kill()
        except Exception:
            pass


def get_sudo_password_once(args: argparse.Namespace) -> str | None:
    needs_sudo = args.enable_tap or (args.enable_tcpdump and args.tcpdump_use_sudo)
    if not needs_sudo:
        return None
    password = os.environ.get("SUDO_PASSWORD")
    if password:
        return password
    return getpass.getpass("Sudo password for ns-3 TAP/tcpdump: ")


def request_sionna_radio_map_export(args: argparse.Namespace, timeout: float = 600.0) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(b"EXPORT_RADIO_MAP", ("127.0.0.1", args.sionna_port))
        response, _address = sock.recvfrom(1024)
        return response.decode(errors="ignore").startswith("EXPORT_RADIO_MAP_DONE")
    except OSError:
        return False
    finally:
        sock.close()


def coord_dir_name(i: int, row: dict) -> str:
    clean = lambda v: str(v).replace("-", "m").replace(".", "p")
    return f"coord_{i:03d}_x_{clean(row['x'])}_y_{clean(row['y'])}_z_{clean(row['z'])}"


def run_one(args: argparse.Namespace, coord: dict, i: int, run_root: Path, sudo_password: str | None = None) -> dict:
    cdir = run_root / coord_dir_name(i, coord)
    raw, parsed = cdir / "raw", cdir / "parsed"
    raw.mkdir(parents=True)
    parsed.mkdir()
    before_sionna_rows = count_csv_rows(args.radio_map_summary_csv)
    pos = f"{coord['x']},{coord['y']},{coord['z']}"
    flowmon = raw / "flowmon.xml"
    ns3_args = [
        f"--transport={args.transport}", f"--tcpVariant={args.tcpVariant}",
        f"--packetSize={args.packetSize}", f"--maxBytes={args.maxBytes}",
        f"--appStart={args.appStart}", f"--appStop={args.appStop}", f"--simTime={args.simTime}",
        f"--port={args.port}", f"--sionna={args.sionna}", f"--shutdownSionna={args.shutdownSionna}",
        f"--enableTap={'true' if args.enable_tap else 'false'}",
        f"--tapMode={args.tap_mode}",
        f"--tapName={args.tap_name}",
        f"--enablePcap={'true' if args.enable_ns3_pcap else 'false'}",
        "--enableSystemTapPcap=false",
        "--enableFlowMonitor=true",
        "--enablePacketSummary=true", "--enableUePositionSequence=false", f"--uePositions={pos}",
        f"--flowMonitorFile={flowmon}", f"--packetSummaryFile={raw / 'packet_summary.csv'}",
        f"--pcapPrefix={raw / 'ns3_csma'}",
        f"--tcpCwndFile={raw / 'tcp_cwnd.csv'}", f"--tcpRttFile={raw / 'tcp_rtt.csv'}",
        f"--tcpRetransmissionsFile={raw / 'tcp_retransmissions.csv'}",
    ]
    cmd = ["./ns3", "run"]
    if args.enable_tap:
        cmd.append("--enable-sudo")
    cmd.append(f"{args.ns3_program} {' '.join(ns3_args)}")
    env = os.environ.copy()
    if sudo_password:
        env["SUDO_PASSWORD"] = sudo_password
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    (raw / "ns3_stdout.log").write_text(result.stdout)
    (raw / "ns3_stderr.log").write_text(result.stderr)

    ns3 = parse_flowmon(flowmon, args.port, args.appStart, args.appStop)
    if args.start_sionna_server and args.sionna == "true" and result.returncode == 0:
        request_sionna_radio_map_export(args)
    sionna_rows_after = read_csv_rows(args.radio_map_summary_csv)
    if len(sionna_rows_after) >= before_sionna_rows:
        sionna_new = sionna_rows_after[before_sionna_rows:]
    else:
        sionna_new = sionna_rows_after
    sionna = find_sionna_row_for_coord(sionna_rows_after, coord) or (sionna_new[-1] if sionna_new else {})
    sionna_artifacts = {}
    sionna_visualization = {
        "sionna_scene_generated": False,
        "sionna_grid_generated": False,
        "scene_resolution": "",
        "grid_resolution": "",
        "sumo_path_overlay_applied": False,
        "tx_marker_drawn": False,
        "current_rx_marker_drawn": False,
        "camera_mode": args.radio_map_camera_mode,
        "birdseye_camera_used": args.radio_map_camera_mode == "birdseye",
        "warnings": [],
    }
    if sionna:
        write_rows(parsed / "sionna_metrics.csv", [sionna])
        artifact_map = {
            "metadata_json": "sionna_raw.json",
            "scene_png": "sionna_scene.png",
            "grid_png": "sionna_grid.png",
            "stats_png": "sionna_stats.png",
            "path_gain_db_npy": "sionna_path_gain_db.npy",
        }
        for column, filename in artifact_map.items():
            source = sionna.get(column)
            if source and Path(source).exists():
                shutil.copy2(source, raw / filename)
                sionna_artifacts[filename] = str(raw / filename)
        metadata_source = raw / "sionna_raw.json"
        if metadata_source.exists():
            try:
                sionna_meta = json.loads(metadata_source.read_text(encoding="utf-8"))
                visualization_meta = sionna_meta.get("visualization", {})
                sionna_visualization.update({
                    "sionna_scene_generated": bool((raw / "sionna_scene.png").exists()),
                    "sionna_grid_generated": bool((raw / "sionna_grid.png").exists()),
                    "scene_resolution": visualization_meta.get("scene_resolution", ""),
                    "grid_resolution": visualization_meta.get("grid_resolution", ""),
                    "sumo_path_overlay_applied": bool(visualization_meta.get("route_overlay_applied")),
                    "tx_marker_drawn": bool(visualization_meta.get("tx_marker_drawn")),
                    "current_rx_marker_drawn": bool(visualization_meta.get("current_rx_marker_drawn")),
                    "camera_mode": visualization_meta.get("camera_mode", sionna_visualization["camera_mode"]),
                    "birdseye_camera_used": visualization_meta.get("camera_mode") == "birdseye",
                })
                sionna_visualization["warnings"].extend(visualization_meta.get("warnings", []))
            except (OSError, json.JSONDecodeError) as exc:
                sionna_visualization["warnings"].append(f"Could not read Sionna visualization metadata: {exc}")
        else:
            sionna_visualization["warnings"].append("Sionna metadata JSON was not copied")
    else:
        write_rows(parsed / "sionna_metrics.csv", [{"coordinate_index": i, "note": "No new Sionna summary row found"}])
        sionna_visualization["warnings"].append("No new Sionna summary row found")

    cwnd = parse_tcp_trace(raw / "tcp_cwnd.csv", "new_cwnd_bytes")
    rtt = parse_tcp_trace(raw / "tcp_rtt.csv", "new_rtt_ms")
    states = parse_tcp_trace(raw / "tcp_retransmissions.csv", "new_congestion_state")
    tcp = {
        "retransmissions": len([v for v in states if v > 0]),
        "average_rtt_ms": sum(rtt) / len(rtt) if rtt else "",
        "final_cwnd": cwnd[-1] if cwnd else "",
        "average_cwnd": sum(cwnd) / len(cwnd) if cwnd else "",
    }

    summary = {
        "coordinate_index": i, "time_seconds": coord.get("time_seconds", ""), "vehicle_id": coord.get("vehicle_id", ""),
        "x": coord["x"], "y": coord["y"], "z": coord["z"],
        "transport": args.transport, "tcp_variant": args.tcpVariant,
        **ns3,
        "rss_dbm": float_or_blank(sionna.get("rx_power_dbm")),
        "path_gain_db": float_or_blank(sionna.get("rx_nearest_path_gain_db")),
        "sinr_db": float_or_blank(sionna.get("snr_db")),
        "valid_paths": 1 if sionna.get("los_status") == "True" else "",
        "distance_from_tx_m": float_or_blank(sionna.get("tx_rx_distance_m")),
        "los_available": sionna.get("los_status") not in ("", None),
        "los_present": sionna.get("los_status") == "True",
        "valid_paths_count": 1 if sionna.get("los_status") == "True" else 0 if sionna.get("los_status") in ("False", "None") else "",
        "strongest_path_delay_ns": (
            float(sionna["ray_one_way_delay_s"]) * 1e9
            if float_or_blank(sionna.get("ray_one_way_delay_s")) != "" else ""
        ),
        "sionna_recomputed_for_step": bool(sionna),
        **tcp,
        "returncode": result.returncode,
    }
    write_rows(parsed / "ns3_metrics.csv", [ns3])
    write_rows(parsed / "tcp_metrics.csv", [tcp])
    write_rows(parsed / "paths_metrics.csv", [{"coordinate_index": i, "valid_paths": summary["valid_paths"]}])

    (cdir / "metadata.json").write_text(json.dumps({
        "coordinate": coord, "transport": args.transport, "tcpVariant": args.tcpVariant,
        "packetSize": args.packetSize, "maxBytes": args.maxBytes, "appStart": args.appStart,
        "appStop": args.appStop, "port": args.port,
        "tap": {
            "enabled": args.enable_tap,
            "tapName": args.tap_name,
            "tapMode": args.tap_mode,
            "ns3_pcap_enabled": args.enable_ns3_pcap,
            "ns3_pcap_prefix": str(raw / "ns3_csma"),
        },
        "sionna_summary_rows_seen": len(sionna_new),
        "sionna_artifacts": sionna_artifacts,
        "visualization": sionna_visualization,
        "sionna_row_matching": {
            "matched_by_rx_position": bool(find_sionna_row_for_coord(sionna_rows_after, coord)),
            "radio_map_summary_rows_before": before_sionna_rows,
            "radio_map_summary_rows_after": len(sionna_rows_after),
            "new_rows_seen_by_count": len(sionna_new),
        },
        "tcp_trace_files_generated": {
            "cwnd": bool(cwnd), "rtt": bool(rtt), "retransmissions": bool(states),
        },
    }, indent=2))
    return summary


def main() -> int:
    args = parse_args()
    run_root = new_run_root(args.output_root)
    sumo_path, sumo_meta = generate_sumo_coordinates(args, run_root)
    if args.sumo_config or args.sumo_cnam_route:
        (run_root / "sumo_metadata.json").write_text(json.dumps(sumo_meta, indent=2))
        print(json.dumps(sumo_meta, indent=2))
    if args.verify_sumo_only:
        return 0 if sumo_path else 2
    if (args.sumo_config or args.sumo_cnam_route) and not sumo_path:
        return 2

    coordinate_path = sumo_path or args.coordinates
    if not coordinate_path:
        print("Provide --coordinates or --sumo-config", file=sys.stderr)
        return 2
    coords = read_coordinates(coordinate_path)
    write_rows(run_root / "coordinates.csv", coords)
    if sumo_path:
        raw_root = run_root / "raw"
        raw_root.mkdir(exist_ok=True)
        shutil.copy2(sumo_path, raw_root / "sumo_coordinates.csv")
    (run_root / "metadata.json").write_text(json.dumps(vars(args) | {"coordinate_source": str(coordinate_path)}, default=str, indent=2))

    sionna_process = None
    tcpdump_process = None
    sudo_keepalive_process = None
    sudo_password = None
    tcpdump_meta = {"enabled": args.enable_tcpdump, "status": "not_started"}
    if args.start_sionna_server:
        args.sionna = "true"
        sionna_process = start_sionna_server(args, run_root, sumo_path)
    if args.enable_tcpdump and not args.enable_tap and args.tcpdump_interface != "any":
        print("Warning: --enable-tcpdump without --enable-tap may not see ns-3 data packets on a Linux TAP.", file=sys.stderr)
    sudo_password = get_sudo_password_once(args)
    sudo_keepalive_process = start_sudo_keepalive(args.enable_tap or (args.enable_tcpdump and args.tcpdump_use_sudo), sudo_password)
    tcpdump_process, tcpdump_meta = start_tcpdump(args, run_root, sudo_password)
    (run_root / "tcpdump_metadata.json").write_text(json.dumps(tcpdump_meta, indent=2, default=str))

    try:
        summaries = []
        for i, coord in enumerate(coords):
            print(f"[{i + 1}/{len(coords)}] x={coord['x']} y={coord['y']} z={coord['z']}")
            summaries.append(run_one(args, coord, i, run_root, sudo_password))
        write_rows(run_root / "summary.csv", summaries)
        (run_root / "summary.json").write_text(json.dumps(summaries, indent=2))

        plot_dir = run_root / "plots"
        plot_dir.mkdir()
        trajectory_rows, trajectory_meta = make_trajectory_metrics(coords, summaries, args, sumo_meta)
        write_rows(run_root / "trajectory_samples.csv", coords)
        write_rows(run_root / "trajectory_metrics.csv", trajectory_rows)
        (run_root / "trajectory_metrics.json").write_text(json.dumps(trajectory_rows, indent=2))
        (run_root / "trajectory_metadata.json").write_text(json.dumps(trajectory_meta, indent=2, default=str))
        skipped_plots = plot_trajectory_outputs(plot_dir, trajectory_rows)

        x = [float(r["coordinate_index"]) for r in summaries]
        for key, filename, ylabel in [
            ("throughput_mbps", "throughput_vs_coordinate.png", "Mbps"),
            ("packet_delivery_ratio", "pdr_vs_coordinate.png", "ratio"),
            ("packet_loss_ratio", "packet_loss_vs_coordinate.png", "ratio"),
            ("average_delay_ms", "delay_vs_coordinate.png", "ms"),
            ("average_jitter_ms", "jitter_vs_coordinate.png", "ms"),
            ("rss_dbm", "rss_vs_coordinate.png", "dBm"),
            ("path_gain_db", "path_gain_vs_coordinate.png", "dB"),
            ("sinr_db", "sinr_vs_coordinate.png", "dB"),
            ("valid_paths", "valid_paths_vs_coordinate.png", "count"),
            ("average_cwnd", "tcp_cwnd_vs_coordinate.png", "bytes"),
            ("average_rtt_ms", "tcp_rtt_vs_coordinate.png", "ms"),
        ]:
            if not plot_xy(plot_dir / filename, x, [r.get(key, "") for r in summaries], key, ylabel):
                skipped_plots[filename] = "No varying finite samples available"
        (run_root / "plots_metadata.json").write_text(json.dumps({
            "skipped_plots": skipped_plots,
            "note": "Plots with fewer than two finite points or with a constant y-series are skipped to avoid misleading flat/empty charts.",
        }, indent=2))
    finally:
        tcpdump_meta = stop_tcpdump(tcpdump_process, tcpdump_meta)
        (run_root / "tcpdump_metadata.json").write_text(json.dumps(tcpdump_meta, indent=2, default=str))
        stop_sudo_keepalive(sudo_keepalive_process)
        stop_sionna_server(sionna_process)

    print(f"Results written to {run_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
