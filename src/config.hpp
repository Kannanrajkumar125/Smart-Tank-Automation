// config.hpp - tiny INI parser ("[section]" + "key = value", '#'/';' comments).
#pragma once
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include "controller.hpp"

namespace tank {

class Config {
 public:
  static Config load(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open config file: " + path);
    Config c;
    std::string line, section;
    int n = 0;
    while (std::getline(f, line)) c.parse_line(line, section, ++n);
    return c;
  }
  static Config from_string(const std::string& text) {
    Config c;
    std::string section;
    int n = 0;
    size_t pos = 0;
    while (pos <= text.size()) {
      size_t e = text.find('\n', pos);
      if (e == std::string::npos) e = text.size();
      c.parse_line(text.substr(pos, e - pos), section, ++n);
      pos = e + 1;
    }
    return c;
  }

  double num(const std::string& key, double def) const {
    auto it = kv_.find(key);
    if (it == kv_.end()) return def;
    try {
      size_t used = 0;
      double v = std::stod(it->second, &used);
      if (used != it->second.size()) throw std::invalid_argument("");
      return v;
    } catch (...) {
      throw std::runtime_error("config: '" + key + "' is not a number: " + it->second);
    }
  }
  std::string str(const std::string& key, const std::string& def) const {
    auto it = kv_.find(key);
    return it == kv_.end() ? def : it->second;
  }

  // Build and validate controller settings. Throws on invalid values so a bad
  // config can never silently produce an unsafe controller.
  ControllerConfig controller() const {
    ControllerConfig c;
    c.start_pct = num("thresholds.start_pct", c.start_pct);
    c.stop_pct = num("thresholds.stop_pct", c.stop_pct);
    c.fault_samples = (int)num("sensor.fault_samples", c.fault_samples);
    c.recover_samples = (int)num("sensor.recover_samples", c.recover_samples);
    c.max_step_pct = num("sensor.max_step_pct", c.max_step_pct);
    c.src_empty_samples = (int)num("source.empty_samples", c.src_empty_samples);
    c.src_clear_samples = (int)num("source.clear_samples", c.src_clear_samples);
    c.no_flow_window_ms = (uint64_t)(num("protection.no_flow_window_s", 30) * 1000);
    c.min_rise_pct = num("protection.min_rise_pct", c.min_rise_pct);
    c.max_run_ms = (uint64_t)(num("protection.max_run_s", 1800) * 1000);
    std::string err = c.validate();
    if (!err.empty()) throw std::runtime_error("invalid config: " + err);
    return c;
  }

 private:
  static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r"), b = s.find_last_not_of(" \t\r");
    return a == std::string::npos ? "" : s.substr(a, b - a + 1);
  }
  void parse_line(const std::string& raw, std::string& section, int n) {
    std::string l = trim(raw);
    if (l.empty() || l[0] == '#' || l[0] == ';') return;
    if (l[0] == '[') {
      if (l.back() != ']') throw std::runtime_error("config line " + std::to_string(n) + ": bad section");
      section = trim(l.substr(1, l.size() - 2));
      return;
    }
    size_t eq = l.find('=');
    if (eq == std::string::npos) throw std::runtime_error("config line " + std::to_string(n) + ": expected key = value");
    std::string k = trim(l.substr(0, eq)), v = trim(l.substr(eq + 1));
    size_t c = v.find('#');  // inline comment
    if (c != std::string::npos) v = trim(v.substr(0, c));
    kv_[section.empty() ? k : section + "." + k] = v;
  }
  std::map<std::string, std::string> kv_;
};

}  // namespace tank
