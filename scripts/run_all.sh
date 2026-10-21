#!/usr/bin/env bash
# Regenerates all simulation evidence under logs/ (serial-style log + MQTT trace per scenario).
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p logs
for s in normal source_empty sensor_fault no_flow stuck_sensor; do
  ./tank_sim --scenario "$s" --mqtt-log "logs/mqtt_$s.log" > "logs/serial_$s.log"
  echo "$s: $(grep '^# summary' logs/serial_$s.log)"
done
