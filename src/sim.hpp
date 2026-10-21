// sim.hpp - simulated plant (tank, source, sensor, faults), scenarios and the
// run loop that wires Plant -> Controller -> MQTT. Time is virtual (1 s ticks).
#pragma once
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "config.hpp"
#include "controller.hpp"
#include "mqtt.hpp"

namespace tank {

struct SimParams {
  double capacity_l = 1000, init_pct = 50, pump_lps = 5, demand_lps = 2, noise_pct = 0.1;
  double source_l = 1e6, source_low_l = 50;
};

class Plant {
 public:
  explicit Plant(const SimParams& p)
      : source_l(p.source_l), p_(p), level_l_(p.capacity_l * p.init_pct / 100.0) {}

  double true_pct() const { return 100.0 * level_l_ / p_.capacity_l; }

  void step(double dt, bool pump_on) {
    double in = (pump_on && source_l > 0) ? p_.pump_lps * dt : 0;  // dry pump moves nothing
    if (in > source_l) in = source_l;
    source_l -= in;
    level_l_ += in - p_.demand_lps * dt;
    if (level_l_ < 0) level_l_ = 0;
    if (level_l_ > p_.capacity_l) level_l_ = p_.capacity_l;
  }
  bool dry_running(bool pump_on) const { return pump_on && source_l <= 0; }

  double read_sensor() {
    if (dropout) return NAN;
    if (spike) return spike_value;
    if (stuck) return stuck_value;
    double v = true_pct() + p_.noise_pct * noise();
    return v < 0 ? 0 : (v > 100 ? 100 : v);
  }
  bool read_source_switch() const {
    if (switch_stuck_ok) return false;  // failed switch: never reports empty
    return source_l <= p_.source_low_l;
  }
  void set_stuck(bool on) { stuck = on; stuck_value = true_pct(); }

  bool dropout = false, spike = false, stuck = false, switch_stuck_ok = false;
  double spike_value = 95, stuck_value = 0;
  double source_l;
  SimParams& params() { return p_; }

 private:
  double noise() {  // deterministic LCG in [-1, 1]
    seed_ = seed_ * 6364136223846793005ULL + 1442695040888963407ULL;
    return ((seed_ >> 33) / 2147483648.0) * 2.0 - 1.0;
  }
  SimParams p_;
  double level_l_;
  uint64_t seed_ = 42;
};

struct Action {
  uint64_t t_s;
  std::string what;  // see apply() in run_scenario
  double v;
};
struct Scenario {
  std::string name, description;
  SimParams p;
  uint64_t duration_s;
  std::vector<Action> actions;
};

inline Scenario make_scenario(const std::string& n, const SimParams& base) {
  Scenario s{n, "", base, 0, {}};
  if (n == "normal") {
    s.description = "Normal operation: two fill/drain cycles";
    s.p.init_pct = 20; s.duration_s = 1200;
  } else if (n == "source_empty") {
    s.description = "Source runs low mid-fill -> pump stops, start inhibited, resumes after refill";
    s.p.init_pct = 20; s.p.source_l = 300; s.duration_s = 600;
    s.actions = {{200, "source_l", 2000}};
  } else if (n == "sensor_fault") {
    s.description = "1-2 sample glitch ignored; dropout & out-of-range -> SENSOR_FAULT, auto-recovery";
    s.p.init_pct = 20; s.duration_s = 400;
    s.actions = {{20, "spike_on", 95},  {22, "spike_off", 0},     // 2 samples: debounced
                 {60, "dropout_on", 0}, {75, "dropout_off", 0},   // 15 s: fault
                 {150, "spike_on", 135}, {160, "spike_off", 0}};  // out-of-range: fault
  } else if (n == "no_flow") {
    s.description = "Source dry but low-level switch FAILED (reads OK) -> no-flow lockout, operator reset";
    s.p.init_pct = 20; s.p.source_l = 0; s.duration_s = 300;
    s.actions = {{0, "switch_stuck_on", 0}, {80, "source_l", 2000},
                 {80, "switch_stuck_off", 0}, {100, "reset", 0}};
  } else if (n == "stuck_sensor") {
    s.description = "Level sensor frozen while pumping -> no rise detected -> lockout, reset after repair";
    s.p.init_pct = 20; s.duration_s = 300;
    s.actions = {{10, "stuck_on", 0}, {100, "stuck_off", 0}, {110, "reset", 0}};
  } else {
    throw std::runtime_error("unknown scenario: " + n);
  }
  return s;
}
inline std::vector<std::string> scenario_names() {
  return {"normal", "source_empty", "sensor_fault", "no_flow", "stuck_sensor"};
}

struct RunOptions {
  std::string topic_base = "plant/site1/tank1";
  uint64_t telemetry_interval_s = 10;
  uint64_t log_interval_s = 10;
  double speed = 0;  // 0 = as fast as possible; N = N x real time
};

struct RunStats {
  double max_pct = 0, min_pct = 100;
  uint64_t dry_seconds = 0;  // seconds the pump ran while the source was truly empty
  uint64_t pump_seconds = 0;
  int starts = 0;
  std::map<std::string, int> events;
  State final_state = State::IDLE;
};

inline std::string json_escape(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\';
    o += c;
  }
  return o;
}

inline RunStats run_scenario(const Scenario& sc, Controller& ctrl, MqttPublisher& mqtt,
                             std::ostream& log, const RunOptions& opt) {
  Plant plant(sc.p);
  RunStats st;
  const std::string t_tele = opt.topic_base + "/telemetry", t_evt = opt.topic_base + "/event",
                    t_stat = opt.topic_base + "/status", t_cmd = opt.topic_base + "/cmd";

  CommandHandler on_cmd = [&](const std::string& c) {
    if (c == "reset") ctrl.request_reset();  // other payloads are ignored
  };
  mqtt.set_command_handler(on_cmd);
  mqtt.publish(t_stat, "online", 1, true);
  log << "# scenario: " << sc.name << " - " << sc.description << "\n";

  State prev_state = State::IDLE;
  char buf[320];
  for (uint64_t t = 0; t <= sc.duration_s; ++t) {
    mqtt.set_time_ms(t * 1000);
    for (const auto& a : sc.actions) {
      if (a.t_s != t) continue;
      if (a.what == "source_l") plant.source_l = a.v;
      else if (a.what == "demand") plant.params().demand_lps = a.v;
      else if (a.what == "dropout_on") plant.dropout = true;
      else if (a.what == "dropout_off") plant.dropout = false;
      else if (a.what == "spike_on") { plant.spike = true; plant.spike_value = a.v; }
      else if (a.what == "spike_off") plant.spike = false;
      else if (a.what == "stuck_on") plant.set_stuck(true);
      else if (a.what == "stuck_off") plant.set_stuck(false);
      else if (a.what == "switch_stuck_on") plant.switch_stuck_ok = true;
      else if (a.what == "switch_stuck_off") plant.switch_stuck_ok = false;
      else if (a.what == "reset") {
        if (!mqtt.simulate_rx(t_cmd, "reset")) on_cmd("reset");
      } else throw std::runtime_error("bad action: " + a.what);
      snprintf(buf, sizeof buf, "[%7.1fs] SIM   inject: %s %g", (double)t, a.what.c_str(), a.v);
      log << buf << "\n";
    }

    Input in{t * 1000, plant.read_sensor(), plant.read_source_switch()};
    Output out = ctrl.update(in);

    // events -> log + MQTT (QoS1)
    for (const auto& e : out.events) {
      snprintf(buf, sizeof buf, "[%7.1fs] EVENT %-16s %s", e.t_ms / 1000.0, e.code.c_str(),
               e.detail.c_str());
      log << buf << "\n";
      mqtt.publish(t_evt,
                   fmt("{\"ts_ms\":%llu,\"code\":\"%s\",\"detail\":\"%s\"}",
                       (unsigned long long)e.t_ms, e.code.c_str(), json_escape(e.detail).c_str()),
                   1, false);
      st.events[e.code]++;
      if (e.code == "PUMP_START") st.starts++;
    }

    const bool changed = out.state != prev_state;
    if (t % opt.log_interval_s == 0 || changed) {
      snprintf(buf, sizeof buf,
               "[%7.1fs] LVL=%5.1f%% (true %5.1f%%) PUMP=%-3s STATE=%-12s SRC=%-5s SENSOR=%s",
               (double)t, std::isnan(out.level_pct) ? -1.0 : out.level_pct, plant.true_pct(),
               out.pump_on ? "ON" : "OFF", to_string(out.state),
               out.source_empty ? "EMPTY" : "OK", out.sensor_ok ? "OK" : "BAD");
      log << buf << "\n";
    }
    if (t % opt.telemetry_interval_s == 0 || changed) {
      mqtt.publish(t_tele,
                   fmt("{\"ts_ms\":%llu,\"level_pct\":%s,\"pump\":\"%s\",\"state\":\"%s\","
                       "\"sensor_ok\":%s,\"source_empty\":%s}",
                       (unsigned long long)(t * 1000),
                       std::isnan(out.level_pct) ? "null" : fmt("%.1f", out.level_pct).c_str(),
                       out.pump_on ? "ON" : "OFF", to_string(out.state),
                       out.sensor_ok ? "true" : "false", out.source_empty ? "true" : "false"),
                   0, false);
    }
    prev_state = out.state;

    // physics for the next second, driven by the pump command just issued
    if (plant.dry_running(out.pump_on)) st.dry_seconds++;
    if (out.pump_on) st.pump_seconds++;
    plant.step(1.0, out.pump_on);
    st.max_pct = std::fmax(st.max_pct, plant.true_pct());
    st.min_pct = std::fmin(st.min_pct, plant.true_pct());
    st.final_state = out.state;
    if (opt.speed > 0)
      std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 / opt.speed)));
  }
  mqtt.publish(t_stat, "offline", 1, true);
  return st;
}

inline SimParams sim_params_from(const Config& c) {
  SimParams p;
  p.capacity_l = c.num("sim.capacity_l", p.capacity_l);
  p.pump_lps = c.num("sim.pump_lps", p.pump_lps);
  p.demand_lps = c.num("sim.demand_lps", p.demand_lps);
  p.noise_pct = c.num("sim.noise_pct", p.noise_pct);
  p.source_low_l = c.num("sim.source_low_l", p.source_low_l);
  return p;
}

}  // namespace tank
