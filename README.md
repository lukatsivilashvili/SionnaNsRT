# Run The Project

This run uses the `ns3_to_tap` direction:

```text
ns-3 TCP app -> Sionna-managed LTE channel -> ns-3 output side -> tap0 -> Linux host
```

Run:

```bash
cd /home/luksona/WorkingFiles/sionna_files/SionnaNsRT

python run_coordinate_experiments.py \
  --sumo-cnam-route \
  --start-sionna-server \
  --enableTap true \
  --tapName tap0 \
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

Verify:

```bash
latest=$(find results -maxdepth 1 -type d -name 'sionna_ns3_experiment_*' | sort | tail -n 1)

cat "$latest/raw/ns3_to_tap_topology.json"
cat "$latest/host_tcp_sink_metadata.json"
head "$latest/raw/packet_summary.csv"
cat "$latest/tcpdump_metadata.json"
ls -lh "$latest/raw/tcpdump_route.pcap"
tcpdump -nn -r "$latest/raw/tcpdump_route.pcap" | head -30
```

Live host observation, while the simulation is running:

```bash
sudo tcpdump -i tap0 -n -vv tcp port 9000
```

`tap0` is created once by the Python runner and reused with ns-3 `TapBridge` in `UseLocal` mode. The whole SUMO trajectory now runs inside one ns-3 process, so the TCP session is not restarted for every coordinate.

The runner also sets `tap0` and the ns-3 output device to the same MAC address, `02:00:00:00:02:02`. This is required so the Linux TCP stack accepts packets arriving on `tap0`, not only tcpdump.

The runner installs the host return route through the ns-3 PGW/output side:

```bash
sudo ip route replace 7.0.0.0/8 via 10.10.2.1 dev tap0
```

Do not use `7.0.0.0/8 dev tap0` without `via 10.10.2.1`; that makes Linux ARP directly for the UE address `7.0.0.2`, which will not answer.

The Python runner starts its own host TCP sink on `10.10.2.2:9000`, so do not also run `nc -l 9000`; that would occupy the port and prevent the built-in receiver from answering ns-3's TCP SYN.
