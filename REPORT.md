# Technical Report – Smart Tank & Pump Automation

## 1. Goal
Keep a tank between 25 % and 90 % by switching a pump, never run the pump dry, fail
safe when the level sensor is unreliable, and report state and events over MQTT.
Everything is validated against a simulated plant.

## 2. Architecture
Three layers with narrow interfaces:

1. **Controller** (`controller.hpp`) – a deterministic function
   `update(now, level, source_empty) → pump_on, state, events`. No I/O, no clock, no
   globals, so every rule is unit-testable and the file could move to an MCU unchanged.
2. **Plant simulator** (`sim.hpp`) – tank volume, pump flow, consumption, finite source,
   noisy sensor, injectable faults (dropout, spike, stuck, failed source switch).
3. **Transport** (`mqtt.hpp`) – an interface with a mock (trace file) and a libmosquitto
   backend. The controller never depends on connectivity.

## 3. Control rules
* **Hysteresis:** start when level < 25 %, stop when ≥ 90 %. Inside the band the previous
  state is held, which prevents rapid cycling.
* **Sensor validation** – a sample is bad if NaN, outside 0–100 %, or differs from the last
  trusted sample by > 20 % (a tank cannot change that fast). Faults are **debounced**: 3
  consecutive bad samples → `SENSOR_FAULT`, pump forced OFF, no start allowed. Recovery needs
  5 consecutive good samples, and the plausibility baseline is reset (the level may have
  legitimately changed while blind). Debouncing means a 1–2 sample glitch neither stops the
  pump nor raises a fault (shown in `serial_sensor_fault.log` at t=20 s).
* **Source-empty protection** – a low-level switch on the source. Debounced (3 samples to trip,
  5 to clear). While tripped the pump is stopped and cannot start, even if the tank is nearly
  empty. It recovers automatically because running out of water is an expected condition.
* **Defence in depth against a failed source switch** – a *no-flow watchdog*: while the pump
  runs, the level must rise ≥ 0.5 % per 30 s window. If not, the pump stops and the system
  latches `LOCKOUT` until an operator `reset`. The same mechanism catches a frozen sensor,
  a closed valve or a failed pump. A hard 30 min maximum run time also latches `LOCKOUT`.
  Lockout is latched (not auto-retried) because repeated dry starts are what damage pumps.
* **Fail-safe output:** `pump_on` is true only in `FILLING`; every other state, including
  future ones added by mistake, defaults to OFF. A sensor fault during `LOCKOUT` does not
  clear the latch.
* **Config safety:** thresholds are validated at startup (`0 ≤ start < stop ≤ 100`, positive
  debounce counts), so a typo cannot create an unsafe configuration.

## 4. MQTT design
Retained `status` with Last-Will `offline` lets a dashboard distinguish "pump off" from
"controller dead". Telemetry is QoS 0 (periodic, loss-tolerant, sent on every state change
too); events are QoS 1 (must not be lost). The only inbound command is `reset`; it can only
*clear* a lockout, so a spoofed or replayed message cannot start the pump directly.
Connection is asynchronous: control continues if the broker is down.

## 5. Verification evidence
| Evidence | Result |
|---|---|
| `make test` | 29 tests / 269 checks pass; also pass under ASan + UBSan |
| `normal` | 3 starts, level never exceeded 90.2 %, 0 dry-run seconds |
| `source_empty` | pump stopped 2 s after switch trip (debounce); start blocked while empty; resumed 5 s after refill; **0 dry-run seconds** |
| `sensor_fault` | 2-sample spike ignored; NaN and 135 % each → fault in 3 s, pump OFF, auto-recovery after 5 good samples; 0 dry seconds |
| `no_flow` (switch failed, source empty) | pump stopped after 30 s; **dry-run exposure bounded to 30 s** (vs. unbounded without watchdog); `reset` over MQTT resumes |
| `stuck_sensor` | frozen reading detected as no-rise → lockout; reset after repair |

Raw logs: `logs/serial_*.log`, `logs/mqtt_*.log`.

## 6. Limitations (honest list)
1. **Simulation only.** No real sensor drivers, GPIO, relay, or ADC. Plant model is first-order
   (constant flow/demand, 1 s ticks); real tanks have sloshing, inlet turbulence and varying demand.
2. **Libmosquitto backend is untested** here (library and network unavailable). The mock-trace
   path is fully tested, including the inbound-command path.
3. **Overshoot:** the stop decision happens on a 1 s sample, so the level can exceed 90 % by one
   sample's inflow (≈ 0.2 % in simulation); a high-level float switch is still required as an
   independent hardware overflow protection.
4. **Software-only safety.** Dry-run and overflow protection must also exist as hardware
   interlocks (float switch wired in series with the pump contactor, thermal overload). This
   software must not be the only safeguard.
5. **No persistence:** state is lost on reboot; a latched LOCKOUT clears after a restart.
   A real device should store it in non-volatile memory.
6. **No minimum off-time** (anti-short-cycling) beyond hysteresis; motors usually want one.
7. **No MQTT security configured:** no TLS, minimal authentication. `cmd` should be restricted by
   broker ACLs. No message buffering while offline (telemetry is dropped, events are not queued).
8. **Fixed thresholds in the no-flow check** assume pump inflow ≫ demand. A tank with demand close
   to pump capacity needs a larger window or smaller `min_rise_pct`, and the no-flow
   watchdog could then false-trip.
9. **Single sensor.** A slowly drifting (not frozen) sensor is not detected; redundant sensors or a
   flow meter would cover that.

## 7. Possible extensions
Flow meter / pump-current sensing for direct dry-run detection; retained config over MQTT;
persistent lockout; minimum off/on times; offline event queue; Home-Assistant discovery topics.
