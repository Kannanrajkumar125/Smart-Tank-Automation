// mqtt.hpp - publisher abstraction + simulated broker trace.
// Real broker support (libmosquitto) lives in mqtt_mosquitto.hpp and is
// compiled only with -DUSE_MOSQUITTO.
#pragma once
#include <cstdint>
#include <cstdio>
#include <functional>
#include <ostream>
#include <string>

namespace tank {

using CommandHandler = std::function<void(const std::string& payload)>;

class MqttPublisher {
 public:
  virtual ~MqttPublisher() = default;
  virtual bool publish(const std::string& topic, const std::string& payload, int qos,
                       bool retain) = 0;
  virtual void set_command_handler(CommandHandler h) { handler_ = std::move(h); }
  virtual void set_time_ms(uint64_t) {}
  // Deliver an inbound command. Returns false if this backend cannot simulate RX.
  virtual bool simulate_rx(const std::string&, const std::string&) { return false; }

 protected:
  CommandHandler handler_;
};

// Writes every would-be MQTT message to a stream, in broker-trace style.
class MockMqtt : public MqttPublisher {
 public:
  explicit MockMqtt(std::ostream& os) : os_(os) {}
  bool publish(const std::string& topic, const std::string& payload, int qos,
               bool retain) override {
    char ts[32];
    snprintf(ts, sizeof ts, "[%8.1fs]", now_ms_ / 1000.0);
    os_ << ts << " PUB qos" << qos << (retain ? " retain " : "        ") << topic << " " << payload
        << "\n";
    ++published;
    return true;
  }
  void set_time_ms(uint64_t t) override { now_ms_ = t; }
  bool simulate_rx(const std::string& topic, const std::string& payload) override {
    char ts[32];
    snprintf(ts, sizeof ts, "[%8.1fs]", now_ms_ / 1000.0);
    os_ << ts << " RX  " << topic << " " << payload << "\n";
    if (handler_) handler_(payload);
    return true;
  }
  int published = 0;

 private:
  std::ostream& os_;
  uint64_t now_ms_ = 0;
};

}  // namespace tank
