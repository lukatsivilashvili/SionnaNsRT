import time
import os
import socket
import json
import csv
import numpy as np
from scipy.spatial import cKDTree
import subprocess, signal
import argparse
import torch

# Import sionna.rt before TensorFlow. Importing TensorFlow first can crash
# Mitsuba/DrJit initialization on this Python 3.12 environment.
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

from sionna.rt import load_scene, PlanarArray, Transmitter, Receiver, PathSolver, RadioMapSolver, Camera


def tensor_to_numpy(value):
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.array(value)


def json_safe(value):
    """Convert Sionna/Mitsuba/NumPy values into JSON-native values."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if hasattr(value, "numpy"):
        return json_safe(value.numpy())
    try:
        return float(value)
    except (TypeError, ValueError):
        return value


def scalar_float(value):
    value = json_safe(value)
    if isinstance(value, list):
        if not value:
            raise ValueError("Cannot convert empty list to scalar float")
        value = value[0]
    return float(value)


def ensure_antennas(sionna_structure):
    sionna_structure["scene"].tx_array = sionna_structure["planar_array"]
    sionna_structure["scene"].rx_array = sionna_structure["planar_array"]

    for car_id in sionna_structure["sionna_location_db"]:
        tx_antenna_name = f"car_{car_id}_tx_antenna"
        rx_antenna_name = f"car_{car_id}_rx_antenna"
        car_position = np.array(
            [sionna_structure["sionna_location_db"][car_id]['x'], sionna_structure["sionna_location_db"][car_id]['y'],
             sionna_structure["sionna_location_db"][car_id]['z']])
        tx_position = car_position + np.array(sionna_structure["antenna_displacement"])
        rx_position = car_position + np.array(sionna_structure["antenna_displacement"])

        if sionna_structure["scene"].get(tx_antenna_name) is None:
            sionna_structure["scene"].add(Transmitter(tx_antenna_name, position=tx_position, orientation=[0, 0, 0]))
            if sionna_structure["verbose"]:
                print(f"Added TX antenna for car_{car_id}: {tx_antenna_name}")
        else:
            sionna_structure["scene"].get(tx_antenna_name).position = tx_position

        if sionna_structure["scene"].get(rx_antenna_name) is None:
            sionna_structure["scene"].add(Receiver(rx_antenna_name, position=rx_position, orientation=[0, 0, 0]))
            if sionna_structure["verbose"]:
                print(f"Added RX antenna for car_{car_id}: {rx_antenna_name}")
        else:
            sionna_structure["scene"].get(rx_antenna_name).position = rx_position


def position_signature(sionna_structure, car_ids):
    signature = []
    for car_id in car_ids:
        position = sionna_structure["sionna_location_db"].get(car_id)
        if position is None:
            return None
        signature.append((
            car_id,
            round(float(position["x"]), 3),
            round(float(position["y"]), 3),
            round(float(position["z"]), 3),
        ))
    return tuple(signature)


def position_to_array(position):
    return np.array([float(position["x"]), float(position["y"]), float(position["z"])])


def radio_map_camera_for_positions(sionna_structure, tx_pos, rx_pos):
    if sionna_structure["radio_map_camera_mode"] == "fixed":
        return (
            list(sionna_structure["radio_map_camera_position"]),
            list(sionna_structure["radio_map_camera_look_at"]),
        )

    tx_array = position_to_array(tx_pos)
    rx_array = position_to_array(rx_pos)

    link_xy = rx_array[:2] - tx_array[:2]
    link_distance = np.linalg.norm(link_xy)
    if link_distance > 1e-6:
        link_direction = np.array([link_xy[0], link_xy[1], 0.0]) / link_distance
        side_direction = np.array([-link_direction[1], link_direction[0], 0.0])
        target_weight = sionna_structure["radio_map_camera_rx_target_weight"]
        target = tx_array * (1.0 - target_weight) + rx_array * target_weight
        camera_distance = max(
            sionna_structure["radio_map_camera_rx_distance"],
            link_distance * sionna_structure["radio_map_camera_distance_scale"],
        )
        camera_height = max(
            sionna_structure["radio_map_camera_height"],
            link_distance * sionna_structure["radio_map_camera_height_scale"],
        )
        camera_position = (
            target
            + link_direction * camera_distance
            + side_direction * sionna_structure["radio_map_camera_side_offset"]
        )
        camera_position[2] = target[2] + camera_height
    else:
        offset = np.array(sionna_structure["radio_map_camera_rx_offset"])
        camera_position = rx_array + offset

    target_weight = sionna_structure["radio_map_camera_rx_target_weight"]
    look_at = tx_array * (1.0 - target_weight) + rx_array * target_weight
    look_at[2] = sionna_structure["radio_map_camera_look_at_height"]

    return camera_position.tolist(), look_at.tolist()


def min_cached_delay_seconds(sionna_structure, tx_car_id, rx_car_id):
    source_key = f"car_{tx_car_id}"
    target_key = f"car_{rx_car_id}"
    cached = sionna_structure["rays_cache"].get(source_key, {}).get(target_key)
    if not cached:
        return None

    values = []
    for delay_set in cached.get("delays", []):
        delay_array = np.asarray(delay_set).reshape(-1)
        delay_array = delay_array[np.isfinite(delay_array) & (delay_array >= 0.0)]
        if delay_array.size:
            values.append(float(np.min(delay_array)))

    if not values:
        return None
    return min(values)


def cached_los_status(sionna_structure, tx_car_id, rx_car_id):
    source_key = f"car_{tx_car_id}"
    target_key = f"car_{rx_car_id}"
    cached = sionna_structure["rays_cache"].get(source_key, {}).get(target_key)
    if not cached:
        return None
    los_values = cached.get("is_los", [])
    if not los_values:
        return None
    return bool(np.any(los_values))


def create_radio_map_stats_plot(
        stats_path,
        path_gain_db,
        centers,
        tx_pos,
        rx_pos,
        tx_name,
        rx_name,
        metrics):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tx_xy = np.array([float(tx_pos["x"]), float(tx_pos["y"])])
    rx_xy = np.array([float(rx_pos["x"]), float(rx_pos["y"])])
    link_xy = rx_xy - tx_xy
    link_distance = np.linalg.norm(link_xy)
    flat_gain = path_gain_db.reshape(-1)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2), gridspec_kw={"width_ratios": [1.6, 1.0]})

    center_xy = centers[:, :, :2].reshape(-1, 2)
    if link_distance > 1e-6:
        direction = link_xy / link_distance
        sample_distances = np.linspace(0.0, link_distance, 80)
        sample_points = tx_xy + sample_distances[:, None] * direction
        _, nearest_indices = cKDTree(center_xy).query(sample_points)
        gain_profile = flat_gain[nearest_indices]
        axes[0].plot(sample_distances, gain_profile, color="#2f7ed8", linewidth=2.0)
        axes[0].scatter([0.0], [gain_profile[0]], color="#333333", s=45, zorder=3, label="TX")
        axes[0].scatter([link_distance], [metrics["rx_path_gain_db"]], color="#d62728", s=60, zorder=3, label="RX")
        axes[0].set_title("Path Gain Along TX-RX Line")
        axes[0].legend(loc="best")
        axes[0].set_xlim(0.0, link_distance)
    else:
        axes[0].text(0.5, 0.5, "TX/RX overlap", ha="center", va="center")
        axes[0].set_title("Path Gain Toward RX")
    axes[0].set_xlabel("Distance from TX (m)")
    axes[0].set_ylabel("Mean path gain (dB)")
    axes[0].grid(True, alpha=0.25)

    ray_delay = metrics["ray_one_way_delay_s"]
    ray_rtt = metrics["ray_rtt_s"]
    los_status = metrics["los_status"]

    summary_lines = [
        f"TX: {tx_name}",
        f"  ({float(tx_pos['x']):.1f}, {float(tx_pos['y']):.1f}, {float(tx_pos['z']):.1f})",
        f"RX: {rx_name}",
        f"  ({float(rx_pos['x']):.1f}, {float(rx_pos['y']):.1f}, {float(rx_pos['z']):.1f})",
        "",
        f"Distance: {metrics['tx_rx_distance_3d_m']:.1f} m",
        f"LOS: {'unknown' if los_status is None else los_status}",
        "",
        f"Path gain: {metrics['rx_path_gain_db']:.1f} dB",
        f"Path loss: {metrics['rx_path_loss_db']:.1f} dB",
        f"RX power:  {metrics['rx_power_dbm']:.1f} dBm",
        f"SNR:       {metrics['snr_db']:.1f} dB",
        "",
        f"Geom RTT: {metrics['geometric_rtt_s'] * 1e9:.1f} ns",
        f"Ray RTT: {'n/a' if ray_rtt is None else f'{ray_rtt * 1e9:.1f} ns'}",
        "",
        f"Map gain mean: {metrics['path_gain_mean_db']:.1f} dB",
        f"Map loss mean: {metrics['path_loss_mean_db']:.1f} dB",
    ]
    axes[1].axis("off")
    axes[1].text(0.0, 0.98, "\n".join(summary_lines), va="top", family="monospace", fontsize=10.5)
    axes[1].set_title("Key Metrics")

    fig.tight_layout()
    fig.savefig(stats_path, dpi=160)
    plt.close(fig)


def write_radio_map_summary(row, sionna_structure):
    csv_path = sionna_structure["radio_map_summary_csv"]
    if csv_path is None:
        csv_path = os.path.join(sionna_structure["radio_map_dir"], "radio_map_summary.csv")

    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    fieldnames = [
        "index",
        "tx_name",
        "rx_name",
        "tx_x",
        "tx_y",
        "tx_z",
        "rx_x",
        "rx_y",
        "rx_z",
        "frequency_hz",
        "bandwidth_hz",
        "cell_size_x_m",
        "cell_size_y_m",
        "samples_per_tx",
        "max_depth",
        "rx_nearest_path_gain_db",
        "rx_path_loss_db",
        "tx_power_dbm",
        "rx_power_dbm",
        "noise_figure_db",
        "noise_floor_dbm",
        "snr_db",
        "path_gain_min_db",
        "path_gain_mean_db",
        "path_gain_max_db",
        "path_loss_min_db",
        "path_loss_mean_db",
        "path_loss_max_db",
        "tx_rx_distance_m",
        "tx_rx_distance_2d_m",
        "geometric_one_way_delay_s",
        "geometric_rtt_s",
        "ray_one_way_delay_s",
        "ray_rtt_s",
        "los_status",
        "camera_mode",
        "camera_x",
        "camera_y",
        "camera_z",
        "look_at_x",
        "look_at_y",
        "look_at_z",
        "scene_png",
        "grid_png",
        "stats_png",
        "path_gain_db_npy",
        "metadata_json",
    ]

    write_header = not sionna_structure["radio_map_summary_csv_initialized"]
    mode = "w" if write_header else "a"
    with open(csv_path, mode, encoding="utf-8", newline="") as summary_file:
        writer = csv.DictWriter(summary_file, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerow({key: json_safe(row.get(key, "")) for key in fieldnames})
    sionna_structure["radio_map_summary_csv_initialized"] = True

    return csv_path


def export_radio_map_if_needed(sionna_structure):
    if not sionna_structure["export_radio_map"]:
        return

    tx_car_id = sionna_structure["radio_map_tx_car_id"]
    rx_car_id = sionna_structure["radio_map_rx_car_id"]
    signature = position_signature(sionna_structure, [tx_car_id, rx_car_id])
    if signature is None or signature in sionna_structure["exported_radio_map_signatures"]:
        return

    tx_name = f"car_{tx_car_id}_tx_antenna"
    rx_name = f"car_{rx_car_id}_rx_antenna"

    ensure_antennas(sionna_structure)

    scene = sionna_structure["scene"]
    tx_names = list(scene.transmitters.keys())
    if tx_name not in tx_names:
        print(f"Radio map export skipped: no transmitter {tx_name} in scene.")
        return

    tx_index = tx_names.index(tx_name)
    out_dir = sionna_structure["radio_map_dir"]
    os.makedirs(out_dir, exist_ok=True)

    export_index = len(sionna_structure["exported_radio_map_signatures"])
    basename = f"radio_map_{export_index:03d}_tx_{tx_name}_rx_{rx_name}"

    print(f"Computing radio map {basename}...")
    t = time.time()
    radio_map = sionna_structure["radio_map_solver"](
        scene=scene,
        center=sionna_structure["radio_map_center"],
        orientation=None,
        size=sionna_structure["radio_map_size"],
        cell_size=sionna_structure["radio_map_cell_size"],
        samples_per_tx=sionna_structure["radio_map_samples"],
        max_depth=sionna_structure["max_depth"],
        los=sionna_structure["los"],
        specular_reflection=sionna_structure["specular_reflection"],
        diffuse_reflection=sionna_structure["diffuse_reflection"],
        refraction=sionna_structure["refraction"],
        seed=sionna_structure["seed"],
    )

    path_gain = tensor_to_numpy(radio_map.transmitter_radio_map(metric="path_gain", tx=tx_index))
    path_gain = np.squeeze(path_gain)
    path_gain_db = 10.0 * np.log10(np.maximum(path_gain, 1e-30))
    centers = tensor_to_numpy(radio_map.cell_centers)

    npy_path = os.path.join(out_dir, basename + "_path_gain_db.npy")
    scene_png_path = os.path.join(out_dir, basename + "_scene.png")
    grid_png_path = os.path.join(out_dir, basename + "_grid.png")
    stats_png_path = os.path.join(out_dir, basename + "_stats.png")
    json_path = os.path.join(out_dir, basename + ".json")
    np.save(npy_path, path_gain_db)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    x = centers[:, :, 0]
    y = centers[:, :, 1]
    extent = [float(np.nanmin(x)), float(np.nanmax(x)), float(np.nanmin(y)), float(np.nanmax(y))]

    tx_pos = sionna_structure["sionna_location_db"][tx_car_id]
    rx_pos = sionna_structure["sionna_location_db"][rx_car_id]

    plt.figure(figsize=(10, 8))
    image = plt.imshow(path_gain_db, origin="lower", extent=extent, cmap="viridis", aspect="equal")
    plt.colorbar(image, label="Path gain (dB)")
    plt.scatter([tx_pos["x"]], [tx_pos["y"]], marker="^", c="red", s=90, label=f"TX {tx_name}")
    plt.scatter([rx_pos["x"]], [rx_pos["y"]], marker="o", c="white", edgecolors="black", s=80, label=f"RX {rx_name}")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(f"Sionna RT radio map: {tx_name} -> {rx_name}")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(grid_png_path, dpi=160)
    plt.close()

    if sionna_structure["radio_map_render_scene"]:
        camera_position, camera_look_at = radio_map_camera_for_positions(sionna_structure, tx_pos, rx_pos)
        camera = Camera(
            position=camera_position,
            look_at=camera_look_at,
        )
        scene.render_to_file(
            camera=camera,
            filename=scene_png_path,
            radio_map=radio_map,
            resolution=tuple(sionna_structure["radio_map_resolution"]),
            num_samples=sionna_structure["radio_map_render_samples"],
            rm_tx=tx_index,
            rm_metric="path_gain",
            rm_db_scale=True,
            rm_vmin=sionna_structure["radio_map_vmin"],
            rm_vmax=sionna_structure["radio_map_vmax"],
            fov=sionna_structure["radio_map_camera_fov"],
            show_devices=True,
            show_orientations=sionna_structure["radio_map_show_orientations"],
        )
    else:
        scene_png_path = ""
        camera_position, camera_look_at = radio_map_camera_for_positions(sionna_structure, tx_pos, rx_pos)

    rx_xy = np.array([float(rx_pos["x"]), float(rx_pos["y"])])
    tx_xy = np.array([float(tx_pos["x"]), float(tx_pos["y"])])
    tx_array = position_to_array(tx_pos)
    rx_array = position_to_array(rx_pos)
    tx_rx_distance_2d = float(np.linalg.norm(rx_xy - tx_xy))
    tx_rx_distance = float(np.linalg.norm(rx_array - tx_array))
    center_xy = centers[:, :, :2].reshape(-1, 2)
    nearest_cell_index = int(np.argmin(np.linalg.norm(center_xy - rx_xy, axis=1)))
    rx_nearest_path_gain_db = float(path_gain_db.reshape(-1)[nearest_cell_index])
    rx_path_loss_db = -rx_nearest_path_gain_db
    tx_power_dbm = sionna_structure["radio_map_tx_power_dbm"]
    rx_power_dbm = tx_power_dbm + rx_nearest_path_gain_db
    frequency_hz = scalar_float(sionna_structure["scene"].frequency)
    bandwidth_hz = scalar_float(sionna_structure["scene"].bandwidth)
    noise_floor_dbm = -174.0 + 10.0 * np.log10(bandwidth_hz) + sionna_structure["radio_map_noise_figure_db"]
    snr_db = rx_power_dbm - noise_floor_dbm
    speed_of_light = 299792458.0
    geometric_one_way_delay_s = tx_rx_distance / speed_of_light
    geometric_rtt_s = 2.0 * geometric_one_way_delay_s
    ray_one_way_delay_s = min_cached_delay_seconds(sionna_structure, tx_car_id, rx_car_id)
    ray_rtt_s = None if ray_one_way_delay_s is None else 2.0 * ray_one_way_delay_s
    los_status = cached_los_status(sionna_structure, tx_car_id, rx_car_id)
    metrics = {
        "tx_rx_distance_3d_m": tx_rx_distance,
        "tx_rx_distance_2d_m": tx_rx_distance_2d,
        "rx_path_gain_db": rx_nearest_path_gain_db,
        "rx_path_loss_db": rx_path_loss_db,
        "tx_power_dbm": tx_power_dbm,
        "rx_power_dbm": rx_power_dbm,
        "noise_figure_db": sionna_structure["radio_map_noise_figure_db"],
        "noise_floor_dbm": noise_floor_dbm,
        "snr_db": snr_db,
        "path_gain_min_db": float(np.nanmin(path_gain_db)),
        "path_gain_mean_db": float(np.nanmean(path_gain_db)),
        "path_gain_max_db": float(np.nanmax(path_gain_db)),
        "path_loss_min_db": float(np.nanmin(-path_gain_db)),
        "path_loss_mean_db": float(np.nanmean(-path_gain_db)),
        "path_loss_max_db": float(np.nanmax(-path_gain_db)),
        "geometric_one_way_delay_s": geometric_one_way_delay_s,
        "geometric_rtt_s": geometric_rtt_s,
        "ray_one_way_delay_s": ray_one_way_delay_s,
        "ray_rtt_s": ray_rtt_s,
        "los_status": los_status,
    }

    if sionna_structure["radio_map_export_stats_plot"]:
        create_radio_map_stats_plot(
            stats_png_path,
            path_gain_db,
            centers,
            tx_pos,
            rx_pos,
            tx_name,
            rx_name,
            metrics,
        )
    else:
        stats_png_path = ""

    metadata = {
        "tx": {"name": tx_name, "position": tx_pos},
        "rx": {"name": rx_name, "position": rx_pos},
        "tx_rx_distance_m": tx_rx_distance,
        "tx_rx_distance_2d_m": tx_rx_distance_2d,
        "frequency_hz": frequency_hz,
        "bandwidth_hz": bandwidth_hz,
        "tx_power_dbm": tx_power_dbm,
        "noise_figure_db": sionna_structure["radio_map_noise_figure_db"],
        "cell_size": list(sionna_structure["radio_map_cell_size"]),
        "size": None if sionna_structure["radio_map_size"] is None else list(sionna_structure["radio_map_size"]),
        "center": None if sionna_structure["radio_map_center"] is None else list(sionna_structure["radio_map_center"]),
        "samples_per_tx": sionna_structure["radio_map_samples"],
        "path_gain_db_npy": npy_path,
        "scene_png": scene_png_path,
        "grid_png": grid_png_path,
        "stats_png": stats_png_path,
        "camera_mode": sionna_structure["radio_map_camera_mode"],
        "camera_position": camera_position,
        "camera_look_at": camera_look_at,
        "metrics": metrics,
    }
    with open(json_path, "w", encoding="utf-8") as metadata_file:
        json.dump(json_safe(metadata), metadata_file, indent=2)

    csv_path = write_radio_map_summary(
        {
            "index": export_index,
            "tx_name": tx_name,
            "rx_name": rx_name,
            "tx_x": tx_pos["x"],
            "tx_y": tx_pos["y"],
            "tx_z": tx_pos["z"],
            "rx_x": rx_pos["x"],
            "rx_y": rx_pos["y"],
            "rx_z": rx_pos["z"],
            "frequency_hz": frequency_hz,
            "bandwidth_hz": bandwidth_hz,
            "cell_size_x_m": sionna_structure["radio_map_cell_size"][0],
            "cell_size_y_m": sionna_structure["radio_map_cell_size"][1],
            "samples_per_tx": sionna_structure["radio_map_samples"],
            "max_depth": sionna_structure["max_depth"],
            "rx_nearest_path_gain_db": rx_nearest_path_gain_db,
            "rx_path_loss_db": rx_path_loss_db,
            "tx_power_dbm": tx_power_dbm,
            "rx_power_dbm": rx_power_dbm,
            "noise_figure_db": sionna_structure["radio_map_noise_figure_db"],
            "noise_floor_dbm": noise_floor_dbm,
            "snr_db": snr_db,
            "path_gain_min_db": metrics["path_gain_min_db"],
            "path_gain_mean_db": metrics["path_gain_mean_db"],
            "path_gain_max_db": metrics["path_gain_max_db"],
            "path_loss_min_db": metrics["path_loss_min_db"],
            "path_loss_mean_db": metrics["path_loss_mean_db"],
            "path_loss_max_db": metrics["path_loss_max_db"],
            "tx_rx_distance_m": tx_rx_distance,
            "tx_rx_distance_2d_m": tx_rx_distance_2d,
            "geometric_one_way_delay_s": geometric_one_way_delay_s,
            "geometric_rtt_s": geometric_rtt_s,
            "ray_one_way_delay_s": ray_one_way_delay_s,
            "ray_rtt_s": ray_rtt_s,
            "los_status": los_status,
            "camera_mode": sionna_structure["radio_map_camera_mode"],
            "camera_x": camera_position[0],
            "camera_y": camera_position[1],
            "camera_z": camera_position[2],
            "look_at_x": camera_look_at[0],
            "look_at_y": camera_look_at[1],
            "look_at_z": camera_look_at[2],
            "scene_png": scene_png_path,
            "grid_png": grid_png_path,
            "stats_png": stats_png_path,
            "path_gain_db_npy": npy_path,
            "metadata_json": json_path,
        },
        sionna_structure,
    )

    sionna_structure["exported_radio_map_signatures"].add(signature)
    print(f"Saved radio map to {scene_png_path or grid_png_path} in {(time.time() - t):.2f}s")
    print(f"Updated radio map summary CSV: {csv_path}")


def manage_location_message(message, sionna_structure):
    t = time.time()
    try:
        # LOC_UPDATE handling
        if sionna_structure["verbose"]:
            print(f"LOC_UPDATE message to handle: {message}")
        
        data = message[len("LOC_UPDATE:"):]
        parts = data.split(",")
        car = int(parts[0].replace("obj", ""))

        new_x = float(parts[1])
        new_y = float(parts[2])
        new_z = float(parts[3]) + 0
        
        new_angle = float(parts[4])

        new_v_x = float(parts[5])
        new_v_y = float(parts[6])
        new_v_z = float(parts[7])

        sionna_structure["SUMO_live_location_db"][car] = {"x": new_x, "y": new_y, "z": new_z, "angle": new_angle, "v_x": new_v_x, "v_y": new_v_y, "v_z": new_v_z}

        # 0 - Is object in sionna_location_db?
        if car in sionna_structure["sionna_location_db"]:
            # Fetch the old values
            old_x = sionna_structure["sionna_location_db"][car]["x"]
            old_y = sionna_structure["sionna_location_db"][car]["y"]
            old_z = sionna_structure["sionna_location_db"][car]["z"]
            old_angle = sionna_structure["sionna_location_db"][car]["angle"]

            if sionna_structure["verbose"]:
                print(f"Found in scenario database - Old position: [{old_x}, {old_y}, {old_z}] - Old angle: {old_angle}")

            # Check if the position or angle has changed by more than the thresholds
            position_changed = (
                    abs(new_x - old_x) >= sionna_structure["position_threshold"]
                    or abs(new_y - old_y) >= sionna_structure["position_threshold"]
                    or abs(new_z - old_z) >= sionna_structure["position_threshold"]
            )
            angle_changed = abs(new_angle - old_angle) >= sionna_structure["angle_threshold"]

            if sionna_structure["verbose"] and (position_changed or angle_changed):
                print(f"Update needed for car_{car} - Position changed: {position_changed} - Angle changed: {angle_changed}")
        
        else:
            # First update ever
            if sionna_structure["verbose"]: 
                print(f"First update ever for car_{car} - No previous values found. Forcing update...")
                print(f"New position requested: [{new_x}, {new_y}, {new_z}] - Angle: {new_angle}")
            
            position_changed = True
            angle_changed = True
        
        # 1 - If needed, update Sionna scenario
        if position_changed or angle_changed:
            sionna_structure["sionna_location_db"][car] = sionna_structure["SUMO_live_location_db"][car]
            # Clear caches upon scenario update
            sionna_structure["path_loss_cache"] = {}
            sionna_structure["rays_cache"] = {}
            if sionna_structure["verbose"]:
                print("Pathloss and rays caches cleared.") 

            # Apply change to the scene
            if sionna_structure["scene"].get(f"car_{car}"):
                from_sionna = sionna_structure["scene"].get(f"car_{car}")
            
                new_orientation = ((360 - new_angle) % 360 + 90)*np.pi/180

                from_sionna.position = [new_x, new_y, new_z]
                from_sionna.orientation = [new_orientation, 0, 0]
                from_sionna.velocity = [new_v_x, new_v_y, new_v_z]
                
                if sionna_structure["verbose"]: 
                    print(f"Updated car_{car} position in the scene.")
            else:
                if sionna_structure["verbose"]:
                    print(f"No car_{car} mesh in the scene; antenna markers will be positioned from LOC_UPDATE.")

            sionna_structure["scene"].remove(f"car_{car}_tx_antenna")
            sionna_structure["scene"].remove(f"car_{car}_rx_antenna")
            if sionna_structure["verbose"]:
                print(f"Removed antennas for car_{car} from the scene.")
        
        if sionna_structure["time_checker"]:
            print(f"Location update took: {(time.time() - t) * 1000} ms")
        return car

    except (IndexError, ValueError) as e:
        print(f"EXCEPTION - Location parsing failed: {e}")
        return None

def match_rays_to_cars(paths, sionna_structure):
    t = time.time()
    matched_paths = {}
    
    # Extract and transpose source and target positions
    targets = paths._tgt_positions.numpy().T
    sources = paths._src_positions.numpy().T 

    # Path parameters
    a_real, a_imag = paths.a
    path_coefficients = a_real.numpy() + 1j * a_imag.numpy()
    delays = paths.tau.numpy()
    interactions = paths.interactions.numpy()
    valid = paths.valid.numpy()

    # Adjust car positions for antenna displacement
    adjusted_car_locs = {
        car_id: {
            "x": car_loc["x"] + sionna_structure["antenna_displacement"][0],
            "y": car_loc["y"] + sionna_structure["antenna_displacement"][1],
            "z": car_loc["z"] + sionna_structure["antenna_displacement"][2],
        }
        for car_id, car_loc in sionna_structure["sionna_location_db"].items()
    }

    car_ids = list(adjusted_car_locs.keys())
    car_positions = np.array([[v["x"], v["y"], v["z"]] for v in adjusted_car_locs.values()])
    car_tree = cKDTree(car_positions)

    # Match sources and targets
    source_dists, source_indices = car_tree.query(sources, distance_upper_bound=sionna_structure["position_threshold"])
    target_dists, target_indices = car_tree.query(targets, distance_upper_bound=sionna_structure["position_threshold"])

    for tx_idx, src_idx in enumerate(source_indices):
        if src_idx == len(car_ids):
            if sionna_structure["verbose"]:
                print(f"Warning - No car within tolerance for source {tx_idx}")
            continue

        matched_source_car_name = f"car_{car_ids[src_idx]}"
        if matched_source_car_name not in matched_paths:
            matched_paths[matched_source_car_name] = {}

        for rx_idx, tgt_idx in enumerate(target_indices):
            if tgt_idx == len(car_ids):
                if sionna_structure["verbose"]:
                    print(f"Warning - No car within tolerance for target {rx_idx} (for source {tx_idx})")
                continue

            matched_target_car_name = f"car_{car_ids[tgt_idx]}"
            if matched_target_car_name not in matched_paths[matched_source_car_name]:
                matched_paths[matched_source_car_name][matched_target_car_name] = {
                    'path_coefficients': [],
                    'delays': [],
                    'is_los': []
                }

            try:
                coeff = path_coefficients[rx_idx, 0, tx_idx, 0, :]
                delay = delays[rx_idx, tx_idx, :]
                valid_mask = valid[rx_idx, tx_idx, :].astype(bool)
                interaction_types = interactions[:, rx_idx, tx_idx, :]  # shape: (5, 12)
                interaction_types_masked = interaction_types[:, valid_mask]  # shape: (5, <=12)
                is_los = np.any(interaction_types_masked == 0)

                matched_paths[matched_source_car_name][matched_target_car_name]['path_coefficients'].append(coeff)
                matched_paths[matched_source_car_name][matched_target_car_name]['delays'].append(delay)
                matched_paths[matched_source_car_name][matched_target_car_name]['is_los'].append(bool(is_los))

            except Exception as e:
                print(f"Error encountered for source {tx_idx}, target {rx_idx}: {e}")
                continue
    print(f"Matching took: {(time.time() - t) * 1000} ms")
    return matched_paths

def compute_rays(sionna_structure):
    t = time.time()

    ensure_antennas(sionna_structure)

    # Compute paths
    paths = sionna_structure["path_solver"](scene=sionna_structure["scene"],
                                            max_depth=sionna_structure["max_depth"],
                                            los=sionna_structure["los"],
                                            specular_reflection=sionna_structure["specular_reflection"],
                                            diffuse_reflection=sionna_structure["diffuse_reflection"],
                                            refraction=sionna_structure["refraction"],
                                            synthetic_array=sionna_structure["synthetic_array"],
                                            seed=sionna_structure["seed"])
    paths.normalize_delays = False

    sionna_structure["paths"] = paths

    if sionna_structure["time_checker"]:
        print(f"Ray tracing took: {(time.time() - t) * 1000} ms")
    
    t = time.time()
    matched_paths = match_rays_to_cars(paths, sionna_structure)
    if sionna_structure["time_checker"]:
        print(f"Matching rays to cars took: {(time.time() - t) * 1000} ms")

    # Iterate over sources in matched_paths
    for src_car_id in sionna_structure["sionna_location_db"]:
        current_source_car_name = f"car_{src_car_id}"
        if current_source_car_name in matched_paths:
            matched_paths_for_source = matched_paths[current_source_car_name]

            # Iterate over targets for the current source
            for trg_car_id in sionna_structure["sionna_location_db"]:
                current_target_car_name = f"car_{trg_car_id}"
                if current_target_car_name != current_source_car_name:  # Skip case where source == target
                    if current_target_car_name in matched_paths_for_source:
                        if current_source_car_name not in sionna_structure["rays_cache"]:
                            sionna_structure["rays_cache"][current_source_car_name] = {}
                        # Cache the matched paths for this source-target pair
                        sionna_structure["rays_cache"][current_source_car_name][current_target_car_name] = \
                            matched_paths_for_source[current_target_car_name]
                        if sionna_structure["verbose"]:
                            print(
                                f"Cached paths for source {current_source_car_name} to target {current_target_car_name}")
                    else:
                        # Force an update if the source or target wasn't matched
                        for car_id in sionna_structure["sionna_location_db"]:
                            car_name = f"car_{car_id}"
                            if sionna_structure["scene"].get(car_name):
                                from_sionna = sionna_structure["scene"].get(car_name)
                                new_position = [sionna_structure["SUMO_live_location_db"][car_id]["x"],
                                                sionna_structure["SUMO_live_location_db"][car_id]["y"],
                                                sionna_structure["SUMO_live_location_db"][car_id]["z"]]
                                from_sionna.position = new_position
                                # Update Sionna location database with new positions
                                sionna_structure["sionna_location_db"][car_id] = {"x": new_position[0],
                                                                                  "y": new_position[1],
                                                                                  "z": new_position[2], "angle":
                                                                                      sionna_structure[
                                                                                          "SUMO_live_location_db"][
                                                                                          car_id]["angle"]}
                                # Update antenna positions
                                if sionna_structure["scene"].get(f"{car_name}_tx_antenna"):
                                    sionna_structure["scene"].get(f"{car_name}_tx_antenna").position = \
                                        [new_position[0] + sionna_structure["antenna_displacement"][0],
                                         new_position[1] + sionna_structure["antenna_displacement"][1],
                                         new_position[2] + sionna_structure["antenna_displacement"][2]]
                                    if sionna_structure["verbose"]:
                                        print(f"Forced update for {car_name} and its TX antenna in the scene.")
                                if sionna_structure["scene"].get(f"{car_name}_rx_antenna"):
                                    sionna_structure["scene"].get(f"{car_name}_rx_antenna").position = \
                                        [new_position[0] + sionna_structure["antenna_displacement"][0],
                                         new_position[1] + sionna_structure["antenna_displacement"][1],
                                         new_position[2] + sionna_structure["antenna_displacement"][2]]
                                    if sionna_structure["verbose"]:
                                        print(f"Forced update for {car_name} and its RX antenna in the scene.")
                            else:
                                if sionna_structure["verbose"]:
                                    print(f"No {car_name} mesh in the scene for forced update; antenna markers remain authoritative.")

                        # Re-do matching with updated locations
                        t = time.time()
                        matched_paths = match_rays_to_cars(paths, sionna_structure)
                        if sionna_structure["time_checker"]:
                            print(f"Matching rays to cars (double exec) took: {(time.time() - t) * 1000} ms")
                        if current_source_car_name not in sionna_structure["rays_cache"]:
                            sionna_structure["rays_cache"][current_source_car_name] = {}
                        if current_target_car_name in matched_paths[current_source_car_name]:
                            sionna_structure["rays_cache"][current_source_car_name][current_target_car_name] = \
                                matched_paths[current_source_car_name][current_target_car_name]

    return None

def get_path_loss(car1_id, car2_id, sionna_structure):
    t = time.time()
    # Was the requested value already calculated?
    if car1_id not in sionna_structure["rays_cache"] or car2_id not in sionna_structure["rays_cache"][car1_id]:
        if sionna_structure["verbose"]:
            print(f"Pathloss calculation requested for {car1_id}-{car2_id}: rays not computed yet.")
        compute_rays(sionna_structure)
    
    if sionna_structure["verbose"]:
        print(f"Pathloss calculation requested for {car1_id}-{car2_id}: rays retreived from cache.")
    
    path_coefficients = sionna_structure["rays_cache"][car1_id][car2_id]["path_coefficients"]

    total_cir = 0
    if len(path_coefficients) > 0:
        # Uncoherent paths summation
        sum_coeffs = np.sum(path_coefficients)
        abs_coeffs = np.abs(sum_coeffs)
        square = abs_coeffs ** 2
        total_cir = square

    # Calculate path loss in dB
    if total_cir > 0:
        path_loss = -10 * np.log10(total_cir)
    else:
        # Handle the case where path loss calculation is not valid
        if sionna_structure["verbose"]:
            print(
                f"Pathloss calculation failed for {car1_id}-{car2_id}: got infinite value (not enough rays). Returning 300 dB.")
        path_loss = 300  # Assign 300 dB for loss cases

    if sionna_structure["time_checker"]:
        print(f"Pathloss calculation took: {(time.time() - t) * 1000} ms")
    export_radio_map_if_needed(sionna_structure)
    return path_loss

def manage_path_loss_request(message, sionna_structure):
    try:
        data = message[len("CALC_REQUEST_PATHGAIN:"):]
        parts = data.split(",")
        car_a_str = parts[0].replace("obj", "")
        car_b_str = parts[1].replace("obj", "")

        # Getting each car_id, the origin is marked as 0 - ns-3 marks a car at [0,0,0] when still outside the simulation
        car_a_id = "origin" if car_a_str == "0" else f"car_{int(car_a_str)}" if car_a_str else "origin"
        car_b_id = "origin" if car_b_str == "0" else f"car_{int(car_b_str)}" if car_b_str else "origin"

        if car_a_id == "origin" or car_b_id == "origin":
            # If any, ignoring path_loss requests from the origin, used for statistical calibration
            path_loss_value = 0
        else:
            t = time.time()
            path_loss_value = get_path_loss(car_a_id, car_b_id, sionna_structure)

        return path_loss_value

    except (ValueError, IndexError) as e:
        print(f"EXCEPTION - Error processing path_loss request: {e}")
        return None

def get_delay(car1_id, car2_id, sionna_structure):
    t = time.time()
    # Check and compute rays only if necessary
    if car1_id not in sionna_structure["rays_cache"] or car2_id not in sionna_structure["rays_cache"][car1_id]:
        compute_rays(sionna_structure)

    delays = np.abs(sionna_structure["rays_cache"][car1_id][car2_id]["delays"])
    delays_flat = delays.flatten()

    # Filter positive values
    positive_values = delays_flat[delays_flat >= 0]

    if positive_values.size > 0:
        min_positive_value = np.min(positive_values)
    else:
        min_positive_value = 1e5

    if sionna_structure["time_checker"]:
        print(f"Delay calculation took: {(time.time() - t) * 1000} ms")
    return min_positive_value

def manage_delay_request(message, sionna_structure):
    try:
        data = message[len("CALC_REQUEST_DELAY:"):]
        parts = data.split(",")
        car_a_str = parts[0].replace("obj", "")
        car_b_str = parts[1].replace("obj", "")

        # Getting each car_id, the origin is marked as 0
        car_a_id = "origin" if car_a_str == "0" else f"car_{int(car_a_str)}" if car_a_str else "origin"
        car_b_id = "origin" if car_b_str == "0" else f"car_{int(car_b_str)}" if car_b_str else "origin"

        if car_a_id == "origin" or car_b_id == "origin":
            # If any, ignoring path_loss requests from the origin, used for statistical calibration
            delay = 0
        else:
            delay = get_delay(car_a_id, car_b_id, sionna_structure)

        return delay

    except (ValueError, IndexError) as e:
        print(f"EXCEPTION - Error processing delay request: {e}")
        return None

def manage_los_request(message, sionna_structure):
    t = time.time()
    try:
        data = message[len("CALC_REQUEST_LOS:"):]
        parts = data.split(",")
        car_a_str = parts[0].replace("obj", "")
        car_b_str = parts[1].replace("obj", "")

        # Getting each car_id, the origin is marked as 0
        car_a_id = "origin" if car_a_str == "0" else f"car_{int(car_a_str)}" if car_a_str else "origin"
        car_b_id = "origin" if car_b_str == "0" else f"car_{int(car_b_str)}" if car_b_str else "origin"

        if car_a_id == "origin" or car_b_id == "origin":
            # If any, ignoring path_loss requests from the origin, used for statistical calibration
            los = 0
        else:
            los = sionna_structure["rays_cache"][car_a_id][car_b_id]["is_los"]

        if sionna_structure["time_checker"]:
            print(f"LOS calculation took: {(time.time() - t) * 1000} ms")
        return los

    except (ValueError, IndexError) as e:
        print(f"EXCEPTION - Error processing LOS request: {e}")
        return None

# Function to kill processes using a specific port
def kill_process_using_port(port, verbose=False):
    try:
        result = subprocess.run(['lsof', '-nP', '-t', f'-i:{port}'], stdout=subprocess.PIPE)
        pids = set()
        for line in result.stdout.decode('utf-8').split('\n'):
            line = line.strip()
            if not line:
                continue
            pid = int(line)
            if pid != os.getpid():
                pids.add(pid)

        for pid in pids:
            os.kill(pid, signal.SIGTERM)
            if verbose:
                print(f"Requested process {pid} using port {port} to terminate")

        if pids:
            time.sleep(0.5)
            for pid in pids:
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    continue
                os.kill(pid, signal.SIGKILL)
                if verbose:
                    print(f"Killed process {pid} using port {port}")
    except Exception as e:
        print(f"Error killing process using port {port}: {e}")

# Configure GPU settings
def configure_gpu(verbose=False, gpus=0):
    if os.getenv("CUDA_VISIBLE_DEVICES") is None:
        gpu_num = gpus
        os.environ["CUDA_VISIBLE_DEVICES"] = f"{gpu_num}"

    cuda_available = torch.cuda.is_available()
    if cuda_available:
        try:
            torch.cuda.init()
        except RuntimeError as e:
            print(e)

    if verbose:
        visible = os.getenv("CUDA_VISIBLE_DEVICES", "<unset>")
        print(f"Configured PyTorch runtime. CUDA_VISIBLE_DEVICES={visible}, cuda_available={cuda_available}")


# Main function to manage initialization and variables
def main():
    # Argument parser setup
    parser = argparse.ArgumentParser(description='ns3-rt - Sionna Server Script: use the following options to configure the server. To apply more specific changes, edit the script directly.')
    # Scenario
    parser.add_argument('--path-to-xml-scenario', type=str, default='scenarios/SionnaCircleScenario/scene.xml',
                        help='Path to the .xml file of the scenario (see Sionna documentation for the creation of custom scenarios)')
    parser.add_argument('--frequency', type=float, help='Frequency of the simulation in Hz', default=5.89e9)
    parser.add_argument('--bw', type=float, help='Bandwidth of the simulation in Hz', default=10e6)
    # Integration
    parser.add_argument('--local-machine', action='store_true',
                        help='Flag to indicate if Sionna and ns3-rt are running on the same machine (locally)')
    parser.add_argument('--port', type=int, help='Port for the UDP socket', default=8103)
    # Ray tracing
    parser.add_argument('--position-threshold', type=float, help='Position threshold for ray tracing', default=3)
    parser.add_argument('--angle-threshold', type=float, help='Angle threshold for ray tracing', default=90)
    parser.add_argument('--max-depth', type=int, help='Maximum depth for ray tracing', default=5)
    parser.add_argument('--max-num-paths-per-src', type=int, help='Maximum number of paths per source', default=1e4)
    parser.add_argument('--samples-per-src', type=int, help='Number of samples per source', default=1e4)
    parser.add_argument('--disable-los', action='store_false', help='Flag to exclude LoS paths')
    parser.add_argument('--disable-specular-reflection', action='store_false', help='Flag to exclude specular reflections')
    parser.add_argument('--disable-diffuse-reflection', action='store_false', help='Flag to exclude diffuse reflections')
    parser.add_argument('--disable-refraction', action='store_false', help='Flag to exclude refraction')
    parser.add_argument('--seed', type=int, help='Seed for random number generation', default=42)
    parser.add_argument('--disable-synthetic-array', action='store_false', help='Flag to disable synthetic array approximation')
    # Other
    parser.add_argument('--verbose', action='store_true', help='[DEBUG] Flag for verbose output')
    parser.add_argument('--time-checker', action='store_true', help='[DEBUG] Flag to check time taken for each operation')
    parser.add_argument('--gpu', type=int, help='Number of GPUs, set 0 to use CPU only (refer to TensorFlow and Sionna documentation)', default=2)
    parser.add_argument('--dynamic-objects-name', type=str, help='Name of the dynamic objects; in the Scenario they must be called e.g., car_id, with id=SUMO ID (only number)', default="car")
    parser.add_argument('--export-radio-map', action='store_true', help='Export a Sionna RT radio-map PNG/NPY whenever the configured TX/RX position pair changes')
    parser.add_argument('--radio-map-dir', type=str, default='radio_map_outputs', help='Directory for exported radio-map files')
    parser.add_argument('--radio-map-tx-car-id', type=int, default=2, help='Sionna car id used as radio-map transmitter, default 2 for the LTE eNB')
    parser.add_argument('--radio-map-rx-car-id', type=int, default=1, help='Sionna car id used as radio-map receiver marker, default 1 for the LTE UE')
    parser.add_argument('--radio-map-cell-size', type=float, nargs=2, default=[5.0, 5.0], metavar=('DX', 'DY'), help='Radio-map cell size in meters')
    parser.add_argument('--radio-map-size', type=float, nargs=2, default=None, metavar=('X', 'Y'), help='Optional radio-map size in meters; omit to cover the scene')
    parser.add_argument('--radio-map-center', type=float, nargs=3, default=None, metavar=('X', 'Y', 'Z'), help='Optional radio-map center; omit to use Sionna scene default')
    parser.add_argument('--radio-map-samples', type=int, default=50000, help='Samples per transmitter for radio-map solver')
    parser.add_argument('--radio-map-summary-csv', type=str, default=None, help='CSV file collecting one row for every exported radio map')
    parser.add_argument('--disable-radio-map-scene-render', action='store_false', dest='radio_map_render_scene', help='Disable native Sionna scene render; still writes grid PNG/NPY/JSON/CSV')
    parser.add_argument('--radio-map-camera-mode', type=str, choices=['rx-orbit', 'fixed'], default='rx-orbit', help='Use a camera anchored near each RX/UE position, or a fixed camera')
    parser.add_argument('--radio-map-camera-position', type=float, nargs=3, default=[220.0, -360.0, 220.0], metavar=('X', 'Y', 'Z'), help='Camera position for native Sionna radio-map renders')
    parser.add_argument('--radio-map-camera-look-at', type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'Z'), help='Camera look-at point for native Sionna radio-map renders')
    parser.add_argument('--radio-map-camera-rx-offset', type=float, nargs=3, default=[-120.0, -180.0, 140.0], metavar=('DX', 'DY', 'DZ'), help='Fallback camera offset from the current RX/UE position when TX and RX overlap')
    parser.add_argument('--radio-map-camera-rx-distance', type=float, default=130.0, help='Minimum camera distance behind the RX/UE, away from TX, when camera mode is rx-orbit')
    parser.add_argument('--radio-map-camera-distance-scale', type=float, default=1.05, help='Additional rx-orbit camera distance as a fraction of TX/RX horizontal distance')
    parser.add_argument('--radio-map-camera-side-offset', type=float, default=-60.0, help='Side offset in meters for rx-orbit camera framing')
    parser.add_argument('--radio-map-camera-height', type=float, default=140.0, help='Camera height above the RX/UE when camera mode is rx-orbit')
    parser.add_argument('--radio-map-camera-height-scale', type=float, default=0.55, help='Minimum rx-orbit camera height as a fraction of TX/RX horizontal distance')
    parser.add_argument('--radio-map-camera-rx-target-weight', type=float, default=0.7, help='Camera look-at weight toward RX in rx-orbit mode; 0.5 is midpoint, 1.0 is RX')
    parser.add_argument('--radio-map-camera-look-at-height', type=float, default=20.0, help='Look-at z coordinate when camera mode is rx-orbit')
    parser.add_argument('--radio-map-camera-fov', type=float, default=78.0, help='Camera field of view in degrees for native Sionna renders')
    parser.add_argument('--radio-map-show-orientations', action='store_true', default=True, help='Show Sionna device orientation arrows in native radio-map renders')
    parser.add_argument('--disable-radio-map-show-orientations', action='store_false', dest='radio_map_show_orientations', help='Hide Sionna device orientation arrows in native radio-map renders')
    parser.add_argument('--disable-radio-map-stats-plot', action='store_false', dest='radio_map_export_stats_plot', help='Disable per-coordinate radio-map statistics plots')
    parser.add_argument('--radio-map-resolution', type=int, nargs=2, default=[756, 570], metavar=('WIDTH', 'HEIGHT'), help='Native Sionna render resolution in pixels')
    parser.add_argument('--radio-map-render-samples', type=int, default=128, help='Native Sionna render samples per pixel')
    parser.add_argument('--radio-map-vmin', type=float, default=-140.0, help='Minimum dB value for native radio-map color scale')
    parser.add_argument('--radio-map-vmax', type=float, default=-40.0, help='Maximum dB value for native radio-map color scale')
    parser.add_argument('--radio-map-tx-power-dbm', type=float, default=23.0, help='TX power used for derived RX power/SNR metrics in radio-map CSV and stats plots')
    parser.add_argument('--radio-map-noise-figure-db', type=float, default=7.0, help='Receiver noise figure used for derived noise-floor/SNR metrics')

    args = parser.parse_args()
    # Scenario
    file_name = args.path_to_xml_scenario
    frequency = args.frequency
    bandwidth = args.bw
    # Integration
    local_machine = args.local_machine
    port = args.port
    # Ray tracing
    position_threshold = args.position_threshold
    angle_threshold = args.angle_threshold
    max_depth = args.max_depth
    max_num_paths_per_src = args.max_num_paths_per_src
    samples_per_src = args.samples_per_src
    los = args.disable_los
    specular_reflection = args.disable_specular_reflection
    diffuse_reflection = args.disable_diffuse_reflection
    refraction = args.disable_refraction
    seed = args.seed
    syntetic_array = args.disable_synthetic_array
    # Other
    verbose = args.verbose
    time_checker = args.time_checker
    gpus = args.gpu
    dynamic_objects_name = args.dynamic_objects_name
    export_radio_map = args.export_radio_map
    radio_map_dir = args.radio_map_dir
    radio_map_tx_car_id = args.radio_map_tx_car_id
    radio_map_rx_car_id = args.radio_map_rx_car_id
    radio_map_cell_size = args.radio_map_cell_size
    radio_map_size = args.radio_map_size
    radio_map_center = args.radio_map_center
    radio_map_samples = args.radio_map_samples
    radio_map_summary_csv = args.radio_map_summary_csv
    radio_map_render_scene = args.radio_map_render_scene
    radio_map_camera_mode = args.radio_map_camera_mode
    radio_map_camera_position = args.radio_map_camera_position
    radio_map_camera_look_at = args.radio_map_camera_look_at
    radio_map_camera_rx_offset = args.radio_map_camera_rx_offset
    radio_map_camera_rx_distance = args.radio_map_camera_rx_distance
    radio_map_camera_distance_scale = args.radio_map_camera_distance_scale
    radio_map_camera_side_offset = args.radio_map_camera_side_offset
    radio_map_camera_height = args.radio_map_camera_height
    radio_map_camera_height_scale = args.radio_map_camera_height_scale
    radio_map_camera_rx_target_weight = args.radio_map_camera_rx_target_weight
    radio_map_camera_look_at_height = args.radio_map_camera_look_at_height
    radio_map_camera_fov = args.radio_map_camera_fov
    radio_map_show_orientations = args.radio_map_show_orientations
    radio_map_export_stats_plot = args.radio_map_export_stats_plot
    radio_map_resolution = args.radio_map_resolution
    radio_map_render_samples = args.radio_map_render_samples
    radio_map_vmin = args.radio_map_vmin
    radio_map_vmax = args.radio_map_vmax
    radio_map_tx_power_dbm = args.radio_map_tx_power_dbm
    radio_map_noise_figure_db = args.radio_map_noise_figure_db

    kill_process_using_port(port, verbose)
    configure_gpu(verbose, gpus)

    sionna_structure = dict()

    sionna_structure["verbose"] = verbose
    sionna_structure["time_checker"] = time_checker

    # Load scene and configure radio settings
    sionna_structure["scene"] = load_scene(filename=file_name, merge_shapes_exclude_regex=dynamic_objects_name)
    sionna_structure["scene"].frequency = frequency
    sionna_structure["scene"].bandwidth = bandwidth
    
    # Edit here the settings for the antennas
    element_spacing = 2.5
    sionna_structure["planar_array"] = PlanarArray(num_rows=1, num_cols=1, vertical_spacing=element_spacing, horizontal_spacing=element_spacing, pattern="iso", polarization="V")
    sionna_structure["antenna_displacement"] = [0, 0, 1.5] # Antenna position wrt car position. Edit needed if each car uses a different mesh
    
    # Scenario update frequency settings
    sionna_structure["position_threshold"] = position_threshold
    sionna_structure["angle_threshold"] = angle_threshold
    
    # Ray tracing settings
    sionna_structure["path_solver"] = PathSolver()
    sionna_structure["synthetic_array"] = syntetic_array
    sionna_structure["max_depth"] = max_depth
    sionna_structure["max_num_paths_per_src"] = max_num_paths_per_src
    sionna_structure["samples_per_src"] = samples_per_src
    sionna_structure["los"] = los
    sionna_structure["specular_reflection"] = specular_reflection
    sionna_structure["diffuse_reflection"] = diffuse_reflection
    sionna_structure["refraction"] = refraction
    sionna_structure["seed"] = seed
    sionna_structure["export_radio_map"] = export_radio_map
    sionna_structure["radio_map_solver"] = RadioMapSolver()
    sionna_structure["radio_map_dir"] = radio_map_dir
    sionna_structure["radio_map_tx_car_id"] = radio_map_tx_car_id
    sionna_structure["radio_map_rx_car_id"] = radio_map_rx_car_id
    sionna_structure["radio_map_cell_size"] = radio_map_cell_size
    sionna_structure["radio_map_size"] = radio_map_size
    sionna_structure["radio_map_center"] = radio_map_center
    sionna_structure["radio_map_samples"] = radio_map_samples
    sionna_structure["radio_map_summary_csv"] = radio_map_summary_csv
    sionna_structure["radio_map_render_scene"] = radio_map_render_scene
    sionna_structure["radio_map_camera_mode"] = radio_map_camera_mode
    sionna_structure["radio_map_camera_position"] = radio_map_camera_position
    sionna_structure["radio_map_camera_look_at"] = radio_map_camera_look_at
    sionna_structure["radio_map_camera_rx_offset"] = radio_map_camera_rx_offset
    sionna_structure["radio_map_camera_rx_distance"] = radio_map_camera_rx_distance
    sionna_structure["radio_map_camera_distance_scale"] = radio_map_camera_distance_scale
    sionna_structure["radio_map_camera_side_offset"] = radio_map_camera_side_offset
    sionna_structure["radio_map_camera_height"] = radio_map_camera_height
    sionna_structure["radio_map_camera_height_scale"] = radio_map_camera_height_scale
    sionna_structure["radio_map_camera_rx_target_weight"] = radio_map_camera_rx_target_weight
    sionna_structure["radio_map_camera_look_at_height"] = radio_map_camera_look_at_height
    sionna_structure["radio_map_camera_fov"] = radio_map_camera_fov
    sionna_structure["radio_map_show_orientations"] = radio_map_show_orientations
    sionna_structure["radio_map_export_stats_plot"] = radio_map_export_stats_plot
    sionna_structure["radio_map_resolution"] = radio_map_resolution
    sionna_structure["radio_map_render_samples"] = radio_map_render_samples
    sionna_structure["radio_map_vmin"] = radio_map_vmin
    sionna_structure["radio_map_vmax"] = radio_map_vmax
    sionna_structure["radio_map_tx_power_dbm"] = radio_map_tx_power_dbm
    sionna_structure["radio_map_noise_figure_db"] = radio_map_noise_figure_db
    sionna_structure["radio_map_summary_csv_initialized"] = False
    sionna_structure["exported_radio_map_signatures"] = set()

    # Caches - do not edit
    sionna_structure["path_loss_cache"] = {}
    sionna_structure["delay_cache"] = {}
    sionna_structure["last_path_loss_requested"] = None

    # Set up UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.settimeout(1.0)
    try:
        if local_machine:
            udp_socket.bind(("127.0.0.1", port))  # Local machine configuration
            if verbose:
                print(f"Expecting UDP messages from ns3-rt on localhost:{port}")
        else:
            udp_socket.bind(("0.0.0.0", port))  # External server configuration
            if verbose:
                print(f"Expecting UDP messages from ns3-rt on UDP/{port}")
    except OSError as e:
        udp_socket.close()
        raise SystemExit(
            f"Unable to bind Sionna UDP port {port}: {e}. "
            f"Stop the old server with: lsof -nP -t -i:{port} | xargs -r kill"
        )

    # Location databases and caches
    sionna_structure["SUMO_live_location_db"] = {}  # Real-time object locations in SUMO
    sionna_structure["sionna_location_db"] = {}  # Object locations in Sionna
    sionna_structure["rays_cache"] = {}  # Cache for ray information
    sionna_structure["path_loss_cache"] = {}  # Cache for path loss values

    print(f"Setup complete. Working at {frequency / 1e9} GHz, bandwidth {bandwidth / 1e6} MHz.")

    try:
        while True:
            # Receive data from the socket. The timeout keeps Ctrl+C responsive
            # while the server is waiting between ns-3 requests.
            try:
                payload, address = udp_socket.recvfrom(1024)
            except socket.timeout:
                continue
            message = payload.decode()

            if verbose:
                print(f"Got new message: {message}")

            if message.startswith("LOC_UPDATE:"):
                updated_car = manage_location_message(message, sionna_structure)
                if updated_car is not None:
                    response = "LOC_CONFIRM:" + "obj" + str(updated_car)
                    udp_socket.sendto(response.encode(), address)

            if message.startswith("CALC_REQUEST_PATHGAIN:"):
                pathloss = manage_path_loss_request(message, sionna_structure)
                if pathloss is not None:
                    response = "CALC_DONE_PATHGAIN:" + str(pathloss)
                    udp_socket.sendto(response.encode(), address)

            if message.startswith("CALC_REQUEST_DELAY:"):
                delay = manage_delay_request(message, sionna_structure)
                if delay is not None:
                    response = "CALC_DONE_DELAY:" + str(delay)
                    udp_socket.sendto(response.encode(), address)

            if message.startswith("CALC_REQUEST_LOS:"):
                los = manage_los_request(message, sionna_structure)
                if los is not None:
                    response = "CALC_DONE_LOS:" + str(los)
                    udp_socket.sendto(response.encode(), address)

            if message.startswith("SHUTDOWN_SIONNA"):
                print("Got SHUTDOWN_SIONNA message. Bye!")
                break
    except KeyboardInterrupt:
        print("Interrupted by user. Shutting down Sionna server.")
    finally:
        udp_socket.close()


# Entry point
if __name__ == "__main__":
    main()
