// Minimal dependency-free test runner. Exit code != 0 on any failure.
#include <cmath>
#include <iostream>
#include <sstream>
#include "sim.hpp"

using namespace tank;

static int g_fail = 0, g_checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) { ++g_fail; std::cerr << "  FAIL " << __FILE__ << ":" << __LINE__ << "  " #cond "\n"; } \
  } while (0)
#define TEST(name) static void name(); struct R_##name { R_##name() { tests().push_back({#name, name}); } } r_##name; static void name()

struct T { const char* n; void (*f)(); };
static std::vector<T>& tests() { static std::vector<T> v; return v; }

// Helper: drive controller with a constant sample for n seconds starting at t0.
struct Rig {
  Controller c;
  uint64_t t = 0;
  Output last;
  explicit Rig(ControllerConfig cfg = {}) : c(cfg) {}
  Output step(double lvl, bool src_empty = false) {
    last = c.update({t, lvl, src_empty});
    t += 1000;
    return last;
  }
  void run(double lvl, int n, bool src = false) { for (int i = 0; i < n; ++i) step(lvl, src); }
  bool has(const char* code) const {
    for (auto& e : last.events) if (e.code == code) return true;
    return false;
  }
};

// ---- FR2: thresholds & hysteresis -------------------------------------
TEST(starts_below_25_not_at_25) {
  Rig r;
  CHECK(!r.step(25.0).pump_on);   // exactly 25 -> no start
  CHECK(!r.step(25.1).pump_on);
  Output o = r.step(24.9);
  CHECK(o.pump_on && o.state == State::FILLING && r.has("PUMP_START"));
}
TEST(stops_at_90_inclusive) {
  Rig r;
  r.step(20);
  for (double v = 21; v < 90; v += 1) CHECK(r.step(v).pump_on);
  CHECK(r.step(89.9).pump_on);
  Output o = r.step(90.0);
  CHECK(!o.pump_on && o.state == State::IDLE && r.has("PUMP_STOP"));
}
TEST(hysteresis_band_holds_state) {
  Rig r;
  r.run(60, 5);                       // idle in the band: must not start
  CHECK(!r.last.pump_on);
  for (double v = 55; v > 25; v -= 5) CHECK(!r.step(v).pump_on);  // draining through the band
  CHECK(!r.step(25.0).pump_on);
  CHECK(r.step(20).pump_on);          // start
  for (int i = 0; i < 20; ++i) CHECK(r.step(24 + i * 2.0).pump_on);  // 24..62 still on
  for (double v = 70; v < 90; v += 5) r.step(v);
  r.step(89.0);
  CHECK(r.last.pump_on);              // still on below 90
  r.step(91.0);
  CHECK(!r.last.pump_on);
  r.run(91, 5);                       // falling but above 25: stays off
  CHECK(!r.last.pump_on);
}

// ---- FR3: sensor fault handling ----------------------------------------
TEST(nan_debounce_then_fault_pump_off) {
  Rig r;
  r.step(20);
  CHECK(r.last.pump_on);
  r.step(NAN); CHECK(r.last.pump_on && r.last.state == State::FILLING);  // 1 bad: hold
  r.step(NAN); CHECK(r.last.pump_on);                                    // 2 bad: hold
  r.step(NAN);                                                           // 3 bad: fault
  CHECK(!r.last.pump_on && r.last.state == State::SENSOR_FAULT && r.has("SENSOR_FAULT"));
  CHECK(!r.last.sensor_ok);
}
TEST(glitch_shorter_than_debounce_ignored) {
  Rig r;
  r.step(40); r.step(135.0); r.step(40);   // 1 bad sample
  CHECK(r.last.state == State::IDLE && r.last.sensor_ok);
  r.step(20); r.step(NAN); r.step(NAN); r.step(21);
  CHECK(r.last.pump_on && r.last.state == State::FILLING);
}
TEST(out_of_range_and_negative_are_faults) {
  Rig r; r.step(50);
  r.run(100.5, 3); CHECK(r.last.state == State::SENSOR_FAULT);
  Rig q; q.step(50);
  q.run(-0.5, 3);  CHECK(q.last.state == State::SENSOR_FAULT);
}
TEST(implausible_jump_rejected) {
  Rig r; r.step(30);
  r.run(80, 3);                      // 30 -> 80 in one sample is physically impossible
  CHECK(r.last.state == State::SENSOR_FAULT);
}
TEST(fault_never_starts_pump_even_if_level_low) {
  Rig r; r.step(50); r.run(NAN, 3);
  for (int i = 0; i < 4; ++i) CHECK(!r.step(10).pump_on);   // 4 good < 5 recover
  CHECK(r.last.state == State::SENSOR_FAULT);
}
TEST(fault_recovers_after_good_samples) {
  Rig r; r.step(50); r.run(NAN, 3);
  r.run(40, 4); CHECK(r.last.state == State::SENSOR_FAULT);
  r.step(40);   CHECK(r.last.state == State::IDLE && r.has("SENSOR_RECOVERED"));
  r.step(20);   CHECK(r.last.pump_on);   // normal control resumes
}
TEST(recovery_rebaselines_after_level_changed) {
  Rig r; r.step(70); r.run(NAN, 3);
  r.run(30, 5);   // level legitimately different after outage: must not be rejected as a jump
  CHECK(r.last.state == State::IDLE);
}

// ---- FR3: source-empty / dry-run protection ----------------------------
TEST(source_empty_stops_running_pump) {
  Rig r; r.step(20); CHECK(r.last.pump_on);
  r.step(30, true); r.step(30, true); CHECK(r.last.pump_on);   // debounce (2 samples)
  r.step(30, true);
  CHECK(!r.last.pump_on && r.last.state == State::SOURCE_EMPTY && r.has("SOURCE_EMPTY") && r.has("PUMP_STOP"));
}
TEST(source_empty_blocks_start) {
  Rig r; r.run(50, 1, true); r.run(50, 2, true);
  for (int i = 0; i < 10; ++i) CHECK(!r.step(5, true).pump_on);   // very low level, still no start
  CHECK(r.last.state == State::SOURCE_EMPTY);
}
TEST(source_flicker_does_not_trip) {
  Rig r; r.step(20);
  for (int i = 0; i < 20; ++i) r.step(30 + i * 0.1, i % 2 == 0);  // alternating -> never 3 in a row
  CHECK(r.last.pump_on);
}
TEST(source_restore_needs_debounce_then_pump_resumes) {
  Rig r; r.step(20); r.run(30, 3, true);
  CHECK(r.last.state == State::SOURCE_EMPTY);
  r.run(15, 4); CHECK(r.last.state == State::SOURCE_EMPTY && !r.last.pump_on);
  r.step(15);   CHECK(r.last.state == State::IDLE && r.has("SOURCE_RESTORED"));
  r.step(15);   CHECK(r.last.pump_on);
}
TEST(no_flow_lockout_latches_until_reset) {
  Rig r;
  r.step(20);
  uint64_t start = r.t;
  while (r.t - start < 29000) CHECK(r.step(20).pump_on);   // inside window: allowed
  r.step(20);                                               // 30 s with no rise
  CHECK(!r.last.pump_on && r.last.state == State::LOCKOUT && r.has("DRY_RUN_LOCKOUT"));
  r.run(5, 100);                                            // latched even at very low level
  CHECK(r.last.state == State::LOCKOUT && !r.last.pump_on);
  r.c.request_reset();
  r.step(20);
  CHECK(r.has("LOCKOUT_RESET") && r.last.pump_on);
}
TEST(rising_level_never_trips_no_flow) {
  Rig r; r.step(20);
  for (int i = 1; i < 69; ++i) CHECK(r.step(20 + i).pump_on || r.last.state == State::IDLE);
  CHECK(r.last.state != State::LOCKOUT);
}
TEST(max_run_time_lockout) {
  ControllerConfig c; c.max_run_ms = 100000; c.no_flow_window_ms = 30000; c.min_rise_pct = 0.01;
  Rig r(c); double v = 20; r.step(v);
  for (int i = 0; i < 110 && r.last.state == State::FILLING; ++i) r.step(v += 0.05);  // slow trickle
  CHECK(r.last.state == State::LOCKOUT && r.last.pump_on == false);
}
TEST(reset_ignored_outside_lockout) {
  Rig r; r.step(50);
  r.c.request_reset(); r.step(50);
  CHECK(r.has("RESET_IGNORED") && r.last.state == State::IDLE && !r.last.pump_on);
}
TEST(lockout_survives_sensor_fault) {
  Rig r; r.step(20);
  for (int i = 0; i < 31; ++i) r.step(20);
  CHECK(r.last.state == State::LOCKOUT);
  r.run(NAN, 5);                                // lockout must stay latched
  CHECK(r.last.state == State::LOCKOUT);
}

// ---- config ---------------------------------------------------------------
TEST(config_parses_and_applies) {
  Config c = Config::from_string("[thresholds]\nstart_pct = 30 # c\nstop_pct=85\n[protection]\nno_flow_window_s=10\n");
  ControllerConfig cc = c.controller();
  CHECK(cc.start_pct == 30 && cc.stop_pct == 85 && cc.no_flow_window_ms == 10000);
}
TEST(config_rejects_unsafe_values) {
  bool threw = false;
  try { Config::from_string("[thresholds]\nstart_pct=90\nstop_pct=25\n").controller(); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try { Config::from_string("[thresholds]\nstart_pct=abc\n").controller(); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
  threw = false;
  try { Config::from_string("[sensor]\nfault_samples=0\n").controller(); } catch (const std::exception&) { threw = true; }
  CHECK(threw);
}
TEST(shipped_config_loads) {
  ControllerConfig cc = Config::load("config/config.ini").controller();
  CHECK(cc.start_pct == 25 && cc.stop_pct == 90);
}

// ---- end-to-end scenarios with safety invariants -----------------------
static RunStats run(const char* name, std::string* mqtt_out = nullptr) {
  Scenario sc = make_scenario(name, SimParams{});
  Controller c(ControllerConfig{});
  std::ostringstream log, mq;
  MockMqtt m(mq);
  RunStats s = run_scenario(sc, c, m, log, RunOptions{});
  if (mqtt_out) *mqtt_out = mq.str();
  return s;
}
TEST(e2e_normal_cycles_and_never_overflows) {
  RunStats s = run("normal");
  CHECK(s.starts >= 1 && s.events["PUMP_STOP"] >= 1);
  CHECK(s.max_pct < 91.0);   // 90% stop + 1 s overshoot
  CHECK(s.dry_seconds == 0);
}
TEST(e2e_source_empty_zero_dry_seconds) {
  RunStats s = run("source_empty");
  CHECK(s.events["SOURCE_EMPTY"] == 1 && s.events["SOURCE_RESTORED"] == 1);
  CHECK(s.dry_seconds == 0 && s.max_pct < 91.0);
}
TEST(e2e_sensor_fault_events) {
  RunStats s = run("sensor_fault");
  CHECK(s.events["SENSOR_FAULT"] == 2 && s.events["SENSOR_RECOVERED"] == 2);  // glitch ignored
  CHECK(s.max_pct < 91.0 && s.dry_seconds == 0);
}
TEST(e2e_failed_switch_dry_run_bounded_by_window) {
  RunStats s = run("no_flow");
  CHECK(s.events["DRY_RUN_LOCKOUT"] == 1 && s.events["LOCKOUT_RESET"] == 1);
  CHECK(s.dry_seconds <= 31);   // no_flow_window (30 s) + 1 tick
}
TEST(e2e_stuck_sensor_locks_out) {
  RunStats s = run("stuck_sensor");
  CHECK(s.events["DRY_RUN_LOCKOUT"] == 1);
}
TEST(e2e_mqtt_topics_and_will) {
  std::string m;
  run("source_empty", &m);
  CHECK(m.find("retain plant/site1/tank1/status online") != std::string::npos);
  CHECK(m.find("plant/site1/tank1/telemetry {\"ts_ms\":") != std::string::npos);
  CHECK(m.find("\"code\":\"SOURCE_EMPTY\"") != std::string::npos);
  CHECK(m.find("status offline") != std::string::npos);
}
TEST(e2e_mqtt_reset_command_path) {
  std::string m;
  run("no_flow", &m);
  CHECK(m.find("RX  plant/site1/tank1/cmd reset") != std::string::npos);
}

int main() {
  for (auto& t : tests()) {
    int before = g_fail;
    t.f();
    std::cout << (g_fail == before ? "[ PASS ] " : "[ FAIL ] ") << t.n << "\n";
  }
  std::cout << "\n" << tests().size() << " tests, " << g_checks << " checks, " << g_fail << " failed\n";
  return g_fail ? 1 : 0;
}
