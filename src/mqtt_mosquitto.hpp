// mqtt_mosquitto.hpp - real MQTT backend (libmosquitto). Build with -DUSE_MOSQUITTO.
// Features: Last-Will "offline" on <base>/status, retained "online" on every
// (re)connect, automatic reconnect with back-off, subscription to <base>/cmd.
// NOTE: written against the libmosquitto 2.x API; it was NOT compiled or run in the
// authoring environment (no libmosquitto / network there) - see README "Limitations".
#pragma once
#ifdef USE_MOSQUITTO
#include <mosquitto.h>
#include <stdexcept>
#include "mqtt.hpp"

namespace tank {

class MosquittoMqtt : public MqttPublisher {
 public:
  MosquittoMqtt(const std::string& host, int port, const std::string& client_id,
                const std::string& user, const std::string& pass, const std::string& base)
      : base_(base) {
    mosquitto_lib_init();
    m_ = mosquitto_new(client_id.c_str(), true, this);
    if (!m_) throw std::runtime_error("mosquitto_new failed");
    if (!user.empty()) mosquitto_username_pw_set(m_, user.c_str(), pass.c_str());
    const std::string st = base_ + "/status";
    mosquitto_will_set(m_, st.c_str(), 7, "offline", 1, true);
    mosquitto_reconnect_delay_set(m_, 1, 30, true);
    mosquitto_connect_callback_set(m_, &MosquittoMqtt::on_connect);
    mosquitto_message_callback_set(m_, &MosquittoMqtt::on_message);
    // connect_async never blocks: the controller keeps running with the broker down.
    if (mosquitto_connect_async(m_, host.c_str(), port, 30) != MOSQ_ERR_SUCCESS)
      throw std::runtime_error("mosquitto_connect_async failed");
    if (mosquitto_loop_start(m_) != MOSQ_ERR_SUCCESS)
      throw std::runtime_error("mosquitto_loop_start failed");
  }
  ~MosquittoMqtt() override {
    publish(base_ + "/status", "offline", 1, true);
    mosquitto_disconnect(m_);
    mosquitto_loop_stop(m_, false);
    mosquitto_destroy(m_);
    mosquitto_lib_cleanup();
  }
  bool publish(const std::string& topic, const std::string& payload, int qos,
               bool retain) override {
    return mosquitto_publish(m_, nullptr, topic.c_str(), (int)payload.size(), payload.data(),
                             qos, retain) == MOSQ_ERR_SUCCESS;  // false while disconnected
  }

 private:
  static void on_connect(struct mosquitto* m, void* self, int rc) {
    if (rc != 0) return;
    auto* s = static_cast<MosquittoMqtt*>(self);
    const std::string cmd = s->base_ + "/cmd", st = s->base_ + "/status";
    mosquitto_subscribe(m, nullptr, cmd.c_str(), 1);
    mosquitto_publish(m, nullptr, st.c_str(), 6, "online", 1, true);
  }
  static void on_message(struct mosquitto*, void* self, const struct mosquitto_message* msg) {
    auto* s = static_cast<MosquittoMqtt*>(self);
    if (s->handler_ && msg->payload)  // runs on network thread: handler must be thread-safe
      s->handler_(std::string((const char*)msg->payload, (size_t)msg->payloadlen));
  }
  struct mosquitto* m_ = nullptr;
  std::string base_;
};

}  // namespace tank
#endif
