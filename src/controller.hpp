// controller.hpp - pure, hardware-independent tank/pump control logic.
// No I/O, no clocks, no globals: time and sensor values come in through
// Input, decisions come out through Output. This makes it unit-testable and
// portable to an MCU (only <cmath>, <string>, <vector> are used).
#pragma once
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace tank {

enum class State {
  IDLE,          // pump off, waiting for level < start threshold
  FILLING,       // pump on, waiting for level >= stop threshold
  SOURCE_EMPTY,  // source low-level switch active: start inhibited / pump stopped
  SENSOR_FAULT,  // level sensor untrustworthy: pump forced off (fail-safe)
  LOCKOUT        // dry-run / no-flow / over-run latched: needs operator reset
};

inline const char* to_string(State s) {
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::FILLING: return "FILLING";
    case State::SOURCE_EMPTY: return "SOURCE_EMPTY";
    case State::SENSOR_FAULT: return "SENSOR_FAULT";
    case State::LOCKOUT: return "LOCKOUT";
  }
  return "?";
}

inline std::string fmt(const char* f, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, f);
  vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

struct ControllerConfig {
  double start_pct = 25.0;     // pump ON when level strictly below this
  double stop_pct = 90.0;      // pump OFF when level at or above this
  int fault_samples = 3;       // consecutive bad samples before SENSOR_FAULT
  int recover_samples = 5;     // consecutive good samples to leave SENSOR_FAULT
  double max_step_pct = 20.0;  // max plausible change between samples
  int src_empty_samples = 3;   // debounce for source-empty switch
  int src_clear_samples = 5;   // debounce for source-restored
  uint64_t no_flow_window_ms = 30000;  // pump must raise level in this window...
  double min_rise_pct = 0.5;           // ...by at least this much
  uint64_t max_run_ms = 1800000;       // absolute continuous run limit

  // Returns "" when valid, otherwise a human-readable problem.
  std::string validate() const {
    if (!(start_pct >= 0 && stop_pct <= 100 && start_pct < stop_pct))
      return "need 0 <= start_pct < stop_pct <= 100";
    if (fault_samples < 1 || recover_samples < 1 || src_empty_samples < 1 ||
        src_clear_samples < 1)
      return "debounce sample counts must be >= 1";
    if (max_step_pct <= 0 || min_rise_pct <= 0) return "max_step/min_rise must be > 0";
    if (no_flow_window_ms == 0 || max_run_ms < no_flow_window_ms)
      return "need 0 < no_flow_window <= max_run";
    return "";
  }
};

struct Input {
  uint64_t now_ms;    // monotonic time
  double level_pct;   // raw sensor reading; NaN = no reading / dropout
  bool source_empty;  // raw source low-level switch
};

struct Event {
  uint64_t t_ms;
  std::string code;  // PUMP_START, PUMP_STOP, SENSOR_FAULT, ...
  std::string detail;
};

struct Output {
  bool pump_on = false;
  State state = State::IDLE;
  bool sensor_ok = true;
  bool source_empty = false;  // debounced
  double level_pct = NAN;     // last trusted level
  std::vector<Event> events;
};

class Controller {
 public:
  explicit Controller(const ControllerConfig& c) : cfg_(c) {}

  // Thread-safe: may be called from an MQTT callback thread. Applied on the
  // next update(). Only clears LOCKOUT; it never forces the pump on.
  void request_reset() { reset_req_.store(true); }

  State state() const { return state_; }

  Output update(const Input& in) {
    events_.clear();
    reason_.clear();
    const uint64_t now = in.now_ms;
    const bool prev_pump = (state_ == State::FILLING);

    // ---- 1. validate the level sample --------------------------------
    bool valid = std::isfinite(in.level_pct) && in.level_pct >= 0.0 && in.level_pct <= 100.0;
    if (!valid) {
      bad_reason_ = "no reading / out of range";
    } else if (have_last_ && std::fabs(in.level_pct - last_valid_) > cfg_.max_step_pct) {
      valid = false;
      bad_reason_ = fmt("implausible jump %.1f%% -> %.1f%%", last_valid_, in.level_pct);
    }
    if (valid) {
      last_valid_ = in.level_pct;
      have_last_ = true;
      valid_cnt_++;
      invalid_cnt_ = 0;
    } else {
      invalid_cnt_++;
      valid_cnt_ = 0;
    }

    // ---- 2. debounce the source-empty switch --------------------------
    if (in.source_empty) { src_empty_cnt_++; src_ok_cnt_ = 0; }
    else { src_ok_cnt_++; src_empty_cnt_ = 0; }
    const bool src_empty = src_empty_cnt_ >= cfg_.src_empty_samples;

    // ---- 3. operator reset (only meaningful in LOCKOUT) ---------------
    if (reset_req_.exchange(false)) {
      if (state_ == State::LOCKOUT) {
        state_ = State::IDLE;
        emit(now, "LOCKOUT_RESET", "operator reset accepted");
      } else {
        emit(now, "RESET_IGNORED", fmt("not in LOCKOUT (state=%s)", to_string(state_)));
      }
    }

    // ---- 4. state machine ---------------------------------------------
    auto lockout = [&](const char* why, const std::string& detail) {
      state_ = State::LOCKOUT;
      reason_ = why;
      emit(now, "DRY_RUN_LOCKOUT", detail);
    };

    if (state_ != State::SENSOR_FAULT && state_ != State::LOCKOUT &&
        invalid_cnt_ >= cfg_.fault_samples) {
      state_ = State::SENSOR_FAULT;
      have_last_ = false;  // re-baseline after recovery
      reason_ = "sensor fault";
      emit(now, "SENSOR_FAULT",
           fmt("%s; %d bad samples; pump forced OFF", bad_reason_.c_str(), invalid_cnt_));
    } else if (state_ == State::SENSOR_FAULT) {
      if (valid_cnt_ >= cfg_.recover_samples) {
        state_ = State::IDLE;
        emit(now, "SENSOR_RECOVERED", fmt("%d consecutive good samples", valid_cnt_));
      }
    } else {
      switch (state_) {
        case State::IDLE:
          if (src_empty) {
            state_ = State::SOURCE_EMPTY;
            emit(now, "SOURCE_EMPTY", "source low-level active; pump start inhibited");
          } else if (valid && in.level_pct < cfg_.start_pct) {
            state_ = State::FILLING;
            run_start_ = mark_ms_ = now;
            mark_level_ = in.level_pct;
            reason_ = fmt("level %.1f%% < %.1f%%", in.level_pct, cfg_.start_pct);
          }
          break;
        case State::FILLING:
          if (src_empty) {
            state_ = State::SOURCE_EMPTY;
            reason_ = "dry-run protection: source empty";
            emit(now, "SOURCE_EMPTY", "source low-level active; pump stopped to avoid dry run");
          } else if (valid) {
            if (in.level_pct >= cfg_.stop_pct) {
              state_ = State::IDLE;
              reason_ = fmt("level %.1f%% >= %.1f%%", in.level_pct, cfg_.stop_pct);
            } else if (now - run_start_ >= cfg_.max_run_ms) {
              lockout("max run time exceeded",
                      fmt("pump ran %llus without reaching stop level",
                          (unsigned long long)((now - run_start_) / 1000)));
            } else if (now - mark_ms_ >= cfg_.no_flow_window_ms) {
              const double rise = in.level_pct - mark_level_;
              if (rise < cfg_.min_rise_pct) {
                lockout("no-flow / suspected dry run",
                        fmt("level rose only %.2f%% in %llus (need %.2f%%); source switch "
                            "or sensor may be failing",
                            rise, (unsigned long long)(cfg_.no_flow_window_ms / 1000),
                            cfg_.min_rise_pct));
              } else {
                mark_ms_ = now;
                mark_level_ = in.level_pct;
              }
            }
          }
          break;
        case State::SOURCE_EMPTY:
          if (src_ok_cnt_ >= cfg_.src_clear_samples) {
            state_ = State::IDLE;
            emit(now, "SOURCE_RESTORED", fmt("%d consecutive OK samples", src_ok_cnt_));
          }
          break;
        case State::LOCKOUT:  // latched until request_reset()
        case State::SENSOR_FAULT:
          break;
      }
    }

    // ---- 5. pump edge events -------------------------------------------
    const bool pump = (state_ == State::FILLING);
    if (pump != prev_pump) emit(now, pump ? "PUMP_START" : "PUMP_STOP", reason_);

    Output o;
    o.pump_on = pump;  // ON only in FILLING: every other state is fail-safe OFF
    o.state = state_;
    o.sensor_ok = (invalid_cnt_ == 0 && state_ != State::SENSOR_FAULT);
    o.source_empty = src_empty;
    o.level_pct = have_last_ ? last_valid_ : NAN;
    o.events = events_;
    return o;
  }

 private:
  void emit(uint64_t t, const char* code, const std::string& d) {
    events_.push_back({t, code, d});
  }

  ControllerConfig cfg_;
  State state_ = State::IDLE;
  std::atomic<bool> reset_req_{false};
  std::vector<Event> events_;
  std::string reason_, bad_reason_;
  double last_valid_ = 0, mark_level_ = 0;
  bool have_last_ = false;
  int valid_cnt_ = 0, invalid_cnt_ = 0, src_empty_cnt_ = 0, src_ok_cnt_ = 0;
  uint64_t run_start_ = 0, mark_ms_ = 0;
};

}  // namespace tank
