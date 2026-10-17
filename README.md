# Smart Tank & Pump Automation

C++17 controller that monitors a (simulated) tank level and pump, starts the pump
below **25 %**, stops it at **90 %**, protects against **dry running** and **sensor
faults**, and publishes telemetry/events over **MQTT**.

```
 Plant (sim) ──level, source switch──▶ Controller (pure logic) ──pump cmd──▶ Plant
                                            │ events/state
                                            ▼
                               MqttPublisher (mock trace | libmosquitto)
```

| File | Purpose |
|---|---|
| `src/controller.hpp` | Hardware-independent state machine (the safety logic) |
| `src/config.hpp`, `config/config.ini` | INI parser + validated configuration |
| `src/mqtt.hpp` | Publisher interface + mock broker trace |
| `src/mqtt_mosquitto.hpp` | Real MQTT backend (`-DUSE_MOSQUITTO`) |
| `src/sim.hpp` | Tank/source/sensor simulator, fault injection, 5 scenarios, run loop |
| `src/main.cpp` | `tank_sim` command-line program |
| `tests/test_main.cpp` | 29 tests (269 checks): unit, config, end-to-end |
| `logs/` | Pre-generated evidence (serial-style logs, MQTT traces, test output) |
| `docs/REPORT.md` | Technical report: design, rationale, limitations |

## Build (no dependencies beyond a C++17 compiler)

```bash
make                    # builds tank_sim and tank_tests
# or: g++ -std=c++17 -O2 -Isrc src/main.cpp -o tank_sim -pthread
# or: cmake -B build && cmake --build build
```

## Test

```bash
make test               # runs ./tank_tests (exit code 0 = all passed)
make sanitize           # same tests under AddressSanitizer + UBSan
```

## Run the simulation

```bash
./tank_sim --scenario normal                         # serial log on stdout, MQTT trace on stderr
./tank_sim --scenario source_empty --mqtt-log /tmp/mqtt.log
./tank_sim --scenario all --mqtt-log logs/mqtt      # one trace file per scenario
make evidence                                        # regenerate everything in logs/
```

Scenarios: `normal`, `source_empty`, `sensor_fault`, `no_flow`, `stuck_sensor`.
Time is virtual (1 s ticks) so a 20-minute scenario runs instantly; use `--speed 1`
for real time, `--duration N` to override length.

## MQTT interface (`<prefix>/<tank_id>` = `plant/site1/tank1` by default)

| Topic | Dir | QoS / retain | Payload |
|---|---|---|---|
| `…/status` | out | 1, retained | `online` / `offline` (broker publishes `offline` via Last Will if we die) |
| `…/telemetry` | out | 0 | `{"ts_ms":..,"level_pct":45.2,"pump":"ON","state":"FILLING","sensor_ok":true,"source_empty":false}` every `telemetry_interval_s` and on every state change; `level_pct` is `null` before the first valid reading |
| `…/event` | out | 1 | `{"ts_ms":..,"code":"PUMP_START","detail":".."}` — codes: `PUMP_START PUMP_STOP SENSOR_FAULT SENSOR_RECOVERED SOURCE_EMPTY SOURCE_RESTORED DRY_RUN_LOCKOUT LOCKOUT_RESET RESET_IGNORED` |
| `…/cmd` | in | 1 | `reset` — clears a latched LOCKOUT only (never starts the pump) |

### Using a real broker

```bash
sudo apt install libmosquitto-dev mosquitto mosquitto-clients
make clean && make MOSQUITTO=1
mosquitto_sub -v -t 'plant/site1/tank1/#' &
./tank_sim --scenario no_flow --mqtt localhost:1883 --speed 10
mosquitto_pub -t plant/site1/tank1/cmd -m reset      # operator reset
```
**Note:** the libmosquitto backend could not be compiled or run where this was
authored (no library, no network). It is small and follows the documented API, but
treat it as untested until you run the commands above.

## Behaviour summary

| State | Pump | Entered when | Leaves when |
|---|---|---|---|
| IDLE | off | boot / level ≥ 90 / recoveries | level < 25 → FILLING |
| FILLING | **on** | level < 25 (strictly) | level ≥ 90 (inclusive); or fault/empty/lockout |
| SOURCE_EMPTY | off | source-empty switch active for 3 samples | switch clear for 5 samples |
| SENSOR_FAULT | off | 3 consecutive NaN / out-of-range / >20 % jumps | 5 consecutive good samples |
| LOCKOUT | off | no level rise ≥ 0.5 % in 30 s of pumping, or run > 30 min | operator `reset` |

Every non-FILLING state outputs pump OFF (fail-safe). All numbers are in `config/config.ini`;
an invalid config (e.g. start ≥ stop) is rejected at startup.

## Evidence in `logs/`

* `serial_<scenario>.log` – what a serial console would show (state every 10 s + all events).
* `mqtt_<scenario>.log` – every MQTT message published/received.
* `test_results.txt` – scenario summaries; `unit_test_results.txt` – test + sanitizer output.
