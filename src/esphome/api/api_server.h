#ifndef ESPHOME_HOMEASSISTANT_API_SERVER_H
#define ESPHOME_HOMEASSISTANT_API_SERVER_H

#include "esphome/defines.h"

#ifdef USE_API

#include "esphome/component.h"
#include "esphome/controller.h"
#include "esphome/api/util.h"
#include "esphome/api/api_message.h"
#include "esphome/api/basic_messages.h"
#include "esphome/api/list_entities.h"
#include "esphome/api/subscribe_state.h"
#include "esphome/api/subscribe_logs.h"
#include "esphome/api/command_messages.h"
#include "esphome/api/service_call_message.h"
#include "esphome/log.h"

#ifdef ARDUINO_ARCH_ESP32
  #include <AsyncTCP.h>
#endif
#ifdef ARDUINO_ARCH_ESP8266
  #include <ESPAsyncTCP.h>
#endif

ESPHOME_NAMESPACE_BEGIN

namespace api {

class APIServer;

class APIConnection {
 public:
  APIConnection(AsyncClient *client, APIServer *parent);
  ~APIConnection();

  void disconnect_client_();
  APIBuffer get_buffer();
  bool send_buffer(APIMessageType type);
  bool send_message(APIMessage &msg);
  bool send_empty_message(APIMessageType type);
  void loop();

#ifdef USE_BINARY_SENSOR
  bool send_binary_sensor_state(binary_sensor::BinarySensor *binary_sensor, bool state);
#endif
#ifdef USE_COVER
  bool send_cover_state(cover::Cover *cover);
#endif
#ifdef USE_FAN
  bool send_fan_state(fan::FanState *fan);
#endif
#ifdef USE_LIGHT
  bool send_light_state(light::LightState *light);
#endif
#ifdef USE_SENSOR
  bool send_sensor_state(sensor::Sensor *sensor, float state);
#endif
#ifdef USE_SWITCH
  bool send_switch_state(switch_::Switch *switch_, bool state);
#endif
#ifdef USE_TEXT_SENSOR
  bool send_text_sensor_state(text_sensor::TextSensor *text_sensor, std::string state);
#endif
  bool send_log_message(int level, const char *tag, const char *line);
  bool send_disconnect_request(const char *reason);
  bool send_ping_request();
  void send_service_call(ServiceCallResponse &call);
#ifdef USE_HOMEASSISTANT_TIME
  void send_time_request();
#endif

 protected:
  friend APIServer;

  void on_error_(int8_t error);
  void on_disconnect_();
  void on_timeout_(uint32_t time);
  void on_data_(uint8_t *buf, size_t len);
  void fatal_error_();
  bool valid_rx_message_type_(uint32_t msg_type);
  void read_message_(uint32_t size, uint32_t type, uint8_t *msg);
  void parse_recv_buffer_();

  // request types
  void on_hello_request_(const HelloRequest &req);
  void on_connect_request_(const ConnectRequest &req);
  void on_disconnect_request_(const DisconnectRequest &req);
  void on_disconnect_response_(const DisconnectResponse &req);
  void on_ping_request_(const PingRequest &req);
  void on_ping_response_(const PingResponse &req);
  void on_device_info_request_(const DeviceInfoRequest &req);
  void on_list_entities_request_(const ListEntitiesRequest &req);
  void on_subscribe_states_request_(const SubscribeStatesRequest &req);
  void on_subscribe_logs_request_(const SubscribeLogsRequest &req);
#ifdef USE_COVER
  void on_cover_command_request_(const CoverCommandRequest &req);
#endif
#ifdef USE_FAN
  void on_fan_command_request_(const FanCommandRequest &req);
#endif
#ifdef USE_LIGHT
  void on_light_command_request_(const LightCommandRequest &req);
#endif
#ifdef USE_SWITCH
  void on_switch_command_request_(const SwitchCommandRequest &req);
#endif
  void on_subscribe_service_calls_request(const SubscribeServiceCallsRequest &req);
  void on_subscribe_home_assistant_states_request(const SubscribeHomeAssistantStatesRequest &req);
  void on_home_assistant_state_response(const HomeAssistantStateResponse &req);

  enum class ConnectionState {
    WAITING_FOR_HELLO,
    WAITING_FOR_CONNECT,
    CONNECTED,
  } connection_state_{ConnectionState::WAITING_FOR_HELLO};

  bool remove_{false};
  AsyncClient *client_;
  APIServer *parent_;

  std::vector<uint8_t> send_buffer_;
  std::vector<uint8_t> recv_buffer_;

  std::string client_info_;
  ListEntitiesIterator list_entities_iterator_;
  InitialStateIterator initial_state_iterator_;

  bool state_subscription_{false};
  int log_subscription_{ESPHOME_LOG_LEVEL_NONE};
  uint32_t last_traffic_;
  bool sent_ping_{false};
  bool service_call_subscription_{false};
};

template<typename T>
class HomeAssistantServiceCallAction;

class APIServer : public Component, public StoringUpdateListenerController {
 public:
  APIServer();
  void setup() override;
  uint16_t get_port() const;
  float get_setup_priority() const override;
  void loop() override;
  void dump_config() override;
  bool check_password(const std::string &password) const;
  bool uses_password() const;
  void set_port(uint16_t port);
  void set_password(const std::string &password);
  void set_reboot_timeout(uint32_t reboot_timeout);
  void handle_disconnect(APIConnection *conn);
#ifdef USE_BINARY_SENSOR
  void on_binary_sensor_update(binary_sensor::BinarySensor *obj, bool state) override;
#endif
#ifdef USE_COVER
  void on_cover_update(cover::Cover *obj) override;
#endif
#ifdef USE_FAN
  void on_fan_update(fan::FanState *obj) override;
#endif
#ifdef USE_LIGHT
  void on_light_update(light::LightState *obj) override;
#endif
#ifdef USE_SENSOR
  void on_sensor_update(sensor::Sensor *obj, float state) override;
#endif
#ifdef USE_SWITCH
  void on_switch_update(switch_::Switch *obj, bool state) override;
#endif
#ifdef USE_TEXT_SENSOR
  void on_text_sensor_update(text_sensor::TextSensor *obj, std::string state) override;
#endif
  void send_service_call(ServiceCallResponse &call);
  template<typename T>
  HomeAssistantServiceCallAction<T> *make_home_assistant_service_call_action();
#ifdef USE_HOMEASSISTANT_TIME
  void request_time();
#endif

  struct HomeAssistantStateSubscription {
    std::string entity_id;
    std::function<void(std::string)> callback;
  };

  void subscribe_home_assistant_state(std::string entity_id, std::function<void(std::string)> f);
  const std::vector<HomeAssistantStateSubscription> &get_state_subs() const;

 protected:
  AsyncServer server_{0};
  uint16_t port_{6053};
  uint32_t reboot_timeout_{300000};
  uint32_t last_connected_{0};
  std::vector<APIConnection *> clients_;
  std::string password_;
  std::vector<HomeAssistantStateSubscription> state_subs_;
};

extern APIServer *global_api_server;

template<typename T>
class HomeAssistantServiceCallAction : public Action<T> {
 public:
  HomeAssistantServiceCallAction(APIServer *parent) : parent_(parent) {}
  void set_service(const std::string &service);
  void set_data(const std::vector<KeyValuePair> &data);
  void set_data_template(const std::vector<KeyValuePair> &data_template);
  void set_variables(const std::vector<TemplatableKeyValuePair> &variables);
  void play(T x) override;
 protected:
  APIServer *parent_;
  ServiceCallResponse resp_;
};

template<typename T>
HomeAssistantServiceCallAction<T> *APIServer::make_home_assistant_service_call_action() {
  return new HomeAssistantServiceCallAction<T>(this);
}

template<typename T>
void HomeAssistantServiceCallAction<T>::set_service(const std::string &service) {
  this->resp_.set_service(service);
}
template<typename T>
void HomeAssistantServiceCallAction<T>::set_data(const std::vector<KeyValuePair> &data) {
  this->resp_.set_data(data);
}
template<typename T>
void HomeAssistantServiceCallAction<T>::set_data_template(const std::vector<KeyValuePair> &data_template) {
  this->resp_.set_data_template(data_template);
}
template<typename T>
void HomeAssistantServiceCallAction<T>::set_variables(const std::vector<TemplatableKeyValuePair> &variables) {
  this->resp_.set_variables(variables);
}
template<typename T>
void HomeAssistantServiceCallAction<T>::play(T x) {
  this->parent_->send_service_call(this->resp_);
  this->play_next(x);
}

} // namespace api

ESPHOME_NAMESPACE_END

#endif //USE_API

#endif //ESPHOME_HOMEASSISTANT_API_SERVER_H
