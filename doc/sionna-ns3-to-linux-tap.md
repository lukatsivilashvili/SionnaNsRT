# Run The Project

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
