// main.cpp - tank_sim: runs a simulated scenario through the controller and
// publishes status/events via MQTT (mock trace by default, real broker with --mqtt).
#include <cstdlib>
#include <memory>
#include <fstream>
#include <iostream>
#include "mqtt_mosquitto.hpp"
#include "sim.hpp"

using namespace tank;

static void usage() {
  std::cerr << "usage: tank_sim [--config FILE] [--scenario NAME|all] [--mqtt-log FILE]\n"
               "                [--duration SEC] [--speed X] [--mqtt HOST:PORT]\n"
               "scenarios: normal source_empty sensor_fault no_flow stuck_sensor\n"
               "--mqtt needs a build with -DUSE_MOSQUITTO\n";
}

int main(int argc, char** argv) {
  std::string cfg_path = "config/config.ini", scen = "normal", mqtt_log, broker;
  long duration = -1;
  double speed = 0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) { usage(); std::exit(2); }
      return argv[++i];
    };
    if (a == "--config") cfg_path = next();
    else if (a == "--scenario") scen = next();
    else if (a == "--mqtt-log") mqtt_log = next();
    else if (a == "--duration") duration = std::atol(next().c_str());
    else if (a == "--speed") speed = std::atof(next().c_str());
    else if (a == "--mqtt") broker = next();
    else { usage(); return 2; }
  }

  try {
    Config cfg = Config::load(cfg_path);
    ControllerConfig cc = cfg.controller();  // throws on unsafe/invalid values
    std::vector<std::string> names = scen == "all" ? scenario_names() : std::vector<std::string>{scen};

    RunOptions opt;
    opt.topic_base = cfg.str("mqtt.topic_prefix", "plant/site1") + "/" + cfg.str("mqtt.tank_id", "tank1");
    opt.telemetry_interval_s = (uint64_t)cfg.num("mqtt.telemetry_interval_s", 10);
    opt.speed = speed;

    for (const auto& n : names) {
      Scenario sc = make_scenario(n, sim_params_from(cfg));
      if (duration >= 0) sc.duration_s = (uint64_t)duration;
      Controller ctrl(cc);

      std::ofstream mf;
      std::unique_ptr<MqttPublisher> mqtt;
      if (!broker.empty()) {
#ifdef USE_MOSQUITTO
        size_t c = broker.rfind(':');
        std::string host = broker.substr(0, c);
        int port = c == std::string::npos ? 1883 : std::atoi(broker.substr(c + 1).c_str());
        mqtt.reset(new MosquittoMqtt(host, port, cfg.str("mqtt.client_id", "tank-controller-01"),
                                     cfg.str("mqtt.username", ""), cfg.str("mqtt.password", ""),
                                     opt.topic_base));
#else
        std::cerr << "built without -DUSE_MOSQUITTO; --mqtt unavailable\n";
        return 2;
#endif
      } else {
        std::ostream* os = &std::cerr;  // default: MQTT trace on stderr, serial log on stdout
        if (!mqtt_log.empty()) {
          std::string path = scen == "all" ? mqtt_log + "." + n : mqtt_log;
          mf.open(path);
          if (!mf) throw std::runtime_error("cannot open " + path);
          os = &mf;
        }
        mqtt.reset(new MockMqtt(*os));
      }

      RunStats s = run_scenario(sc, ctrl, *mqtt, std::cout, opt);
      std::cout << "# summary: " << n << ": max_level=" << fmt("%.1f", s.max_pct)
                << "% min_level=" << fmt("%.1f", s.min_pct) << "% pump_starts=" << s.starts
                << " pump_on_s=" << s.pump_seconds << " dry_run_s=" << s.dry_seconds
                << " final_state=" << to_string(s.final_state) << "\n\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
