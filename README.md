# CNAM SUMO + Sionna RT + ns-3 TCP Experiment

This project runs one managed experiment that combines:

- SUMO mobility for the RX vehicle `car_1`
- Sionna RT channel/radio-map evaluation in the CNAM scene
- ns-3 TCP traffic using the Sionna-managed channel
- Linux-side packet capture with `tcpdump`
- per-coordinate raw data, trajectory metrics, summary CSV/JSON, and plots

The runner is:

```bash
python run_coordinate_experiments.py
```

You do not need to manually start Sionna RT or tcpdump in another terminal. The runner starts and stops them.

## Requirements

Install or verify:

```bash
which sumo
which netconvert
which tcpdump
python3 -c "import traci; print('traci OK')"
```

The ns-3 example must be built:

```bash
cd /home/luksona/WorkingFiles/sionna_files/SionnaNsRT
./ns3 build sionna-ns3-to-linux-tap
```

The Sionna server uses the CNAM scene:

```text
src/sionna/scenarios/cnam_scene_new/cnam_scene.xml
```

## Run Command

Run from the repository root:

```bash
cd /home/luksona/WorkingFiles/sionna_files/SionnaNsRT

python run_coordinate_experiments.py \
  --sumo-cnam-route \
  --start-sionna-server \
  --enable-tap \
  --enable-tcpdump \
  --tcpdump-interface any \
  --output-root results \
  --sumo-max-coordinates 20 \
  --transport tcp \
  --tcpVariant TcpNewReno \
  --packetSize 512 \
  --maxBytes 0 \
  --appStart 1.0 \
  --appStop 20.0 \
  --simTime 20.0 \
  --port 9000
```

The runner asks for the sudo password once because `--enable-tap` and `--enable-tcpdump` need privileged operations. It passes that password to the ns-3 wrapper through `SUDO_PASSWORD`, so you should not get a sudo prompt at every coordinate.

## What Each Flag Does

```text
--sumo-cnam-route
    Generate a small SUMO route for RX vehicle car_1 using the built-in CNAM points.

--start-sionna-server
    Start src/sionna/sionna_v1_server_script.py automatically with the CNAM scene.

--enable-tap
    Enable ns-3 TapBridge so Linux tcpdump can observe TAP-facing packets.

--enable-tcpdump
    Start tcpdump for the whole experiment and save one route-level PCAP.

--tcpdump-interface any
    Capture on Linux interface "any" so tcpdump can observe packets even while the TAP is created by ns-3.

--output-root results
    Store timestamped runs under results/.

--sumo-max-coordinates 20
    Use 20 sampled RX positions from the SUMO route.

--transport tcp
    Use TCP traffic in ns-3.

--tcpVariant TcpNewReno
    Use the ns-3 TCP NewReno congestion-control variant.

--packetSize 512
    TCP BulkSend send size in bytes.

--maxBytes 0
    Send without a byte limit during the application window.

--appStart 1.0
    Start the TCP application at simulation time 1.0 s.

--appStop 20.0
    Stop the TCP application at simulation time 20.0 s.

--simTime 20.0
    Stop each ns-3 coordinate simulation at 20.0 s.

--port 9000
    Use TCP port 9000.
```

## Workflow

The command performs these steps:

1. Creates a timestamped result folder:

```text
results/sionna_ns3_experiment_<timestamp>/
```

2. Generates a SUMO scenario and route for `car_1`.

The built-in route points are defined in `run_coordinate_experiments.py`:

```text
32, -9, 1.5
-72, -31, 1.5
-120, 90, 1.5
-208, -101, 1.5
205, 177, 1.5
```

SUMO creates sampled RX positions and writes:

```text
sumo_coordinates.csv
raw/sumo_coordinates.csv
sumo_metadata.json
```

3. Starts Sionna RT automatically.

The server is started with:

```text
--local-machine
--frequency=2.1e9
--path-to-xml-scenario=scenarios/cnam_scene_new/cnam_scene.xml
--export-radio-map
--radio-map-export-mode=on-request
--radio-map-tx-car-id=2
--radio-map-rx-car-id=1
--radio-map-camera-mode=birdseye
--radio-map-resolution 1920 1080
```

4. Starts tcpdump automatically.

The route-level capture is written to:

```text
raw/tcpdump_route.pcap
tcpdump_metadata.json
raw/tcpdump_stdout.log
raw/tcpdump_stderr.log
```

5. Runs one ns-3 TCP experiment per sampled SUMO coordinate.

For every sampled RX point, the runner:

- sets the ns-3 UE/RX position to that SUMO coordinate,
- runs the TCP application,
- records FlowMonitor output,
- asks Sionna RT to export the radio/channel data for that RX position,
- copies Sionna scene/grid images into that coordinate folder,
- writes parsed metrics.

6. Builds trajectory-level metrics.

Each row in `trajectory_metrics.csv` corresponds to one sampled RX position actually evaluated by Sionna.

7. Generates plots.

Plots with constant or empty series are skipped and recorded in:

```text
plots_metadata.json
```

## Output Structure

Main files:

```text
results/sionna_ns3_experiment_<timestamp>/
  metadata.json
  sumo_metadata.json
  sumo_coordinates.csv
  summary.csv
  summary.json
  trajectory_samples.csv
  trajectory_metrics.csv
  trajectory_metrics.json
  trajectory_metadata.json
  tcpdump_metadata.json
  plots_metadata.json
```

Raw route-level data:

```text
raw/
  sumo_coordinates.csv
  tcpdump_route.pcap
  tcpdump_stdout.log
  tcpdump_stderr.log
```

Plots:

```text
plots/
  rss_vs_step.png
  path_gain_vs_step.png
  sinr_vs_step.png
  rss_vs_distance_along_path.png
  path_gain_vs_distance_along_path.png
  sinr_vs_distance_along_path.png
  trajectory_signal_map.png
```

Per-coordinate folders:

```text
coord_000_x_<x>_y_<y>_z_<z>/
  metadata.json
  raw/
    flowmon.xml
    ns3_stdout.log
    ns3_stderr.log
    packet_summary.csv
    sionna_raw.json
    sionna_scene.png
    sionna_grid.png
    sionna_stats.png
    sionna_path_gain_db.npy
    tcp_cwnd.csv
    tcp_rtt.csv
    tcp_retransmissions.csv
  parsed/
    ns3_metrics.csv
    sionna_metrics.csv
    paths_metrics.csv
    tcp_metrics.csv
```

## Verify The Run

Find the newest run:

```bash
latest=$(find results -maxdepth 1 -type d -name 'sionna_ns3_experiment_*' | sort | tail -n 1)
echo "$latest"
```

Check that SUMO generated the trajectory:

```bash
cat "$latest/sumo_metadata.json"
head "$latest/sumo_coordinates.csv"
```

Check that Sionna evaluated multiple RX positions:

```bash
cat "$latest/trajectory_metadata.json"
head "$latest/trajectory_metrics.csv"
```

Look for:

```json
"sionna_recomputed_per_sampled_position": true
```

Check the packet capture:

```bash
cat "$latest/tcpdump_metadata.json"
ls -lh "$latest/raw/tcpdump_route.pcap"
tcpdump -nn -r "$latest/raw/tcpdump_route.pcap" | head -30
```

Check generated plots:

```bash
find "$latest/plots" -maxdepth 1 -type f | sort
cat "$latest/plots_metadata.json"
```

## Notes

- SUMO is used offline. It generates the RX route first; then ns-3 and Sionna evaluate every sampled coordinate.
- Sionna RT is recomputed per sampled RX coordinate.
- TCP/FlowMonitor plots may be skipped if the TCP metrics are constant across coordinates.
- The most important signal plots are RSS, path gain, SINR, and `trajectory_signal_map.png`.
- The route-level PCAP is captured by Linux tcpdump. Per-coordinate ns-3 raw metrics remain in each `coord_*/raw/` folder.
