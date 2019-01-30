// Auto generated code by esphomeyaml
// ========== AUTO GENERATED INCLUDE BLOCK BEGIN ===========
#include "esphomelib/application.h"
using namespace esphomelib;
// ========== AUTO GENERATED INCLUDE BLOCK END ==========="

void setup() {
  // ===== DO NOT EDIT ANYTHING BELOW THIS LINE =====
  // ========== AUTO GENERATED CODE BEGIN ===========
  App.set_name("test");
  App.set_compilation_datetime(__DATE__ ", " __TIME__);
  LogComponent *logcomponent = App.init_log(115200);
  WiFiComponent *wificomponent = App.init_wifi();
  WiFiAP wifiap = WiFiAP();
  wifiap.set_ssid("FuckGFW");
  wifiap.set_password("refuckgfw");
  wificomponent->add_sta(wifiap);
  // OTAComponent *otacomponent = App.init_ota();
  // otacomponent->set_auth_password("1234567890");
  // otacomponent->start_safe_mode();
  // api::APIServer *api_apiserver = App.init_api_server();
  // api_apiserver->set_password("1234567890");
  // DeepSleepComponent *deepsleepcomponent = App.make_deep_sleep_component();
  // deepsleepcomponent->set_sleep_duration(600000);
  // deepsleepcomponent->set_run_duration(10000);
  ESP32BLETracker *esp32bletracker = App.make_esp32_ble_tracker();
  // esp32bletracker->set_scan_interval(300000);
  // XiaomiDevice *xiaomidevice = esp32bletracker->make_xiaomi_device({0x7A, 0x80, 0x8E, 0x19, 0x36, 0xBA});
  // XiaomiSensor *xiaomisensor = xiaomidevice->make_temperature_sensor("Xiaomi MiJia Temperature");
  // sensor::MQTTSensorComponent *sensor_mqttsensorcomponent = App.register_sensor(xiaomisensor);
  // XiaomiSensor *xiaomisensor_2 = xiaomidevice->make_humidity_sensor("Xiaomi MiJia Humidity");
  // sensor::MQTTSensorComponent *sensor_mqttsensorcomponent_2 = App.register_sensor(xiaomisensor_2);
  // XiaomiSensor *xiaomisensor_3 = xiaomidevice->make_battery_level_sensor("Xiaomi MiJia Battery Level");
  // sensor::MQTTSensorComponent *sensor_mqttsensorcomponent_3 = App.register_sensor(xiaomisensor_3);
  // =========== AUTO GENERATED CODE END ============
  // ========= YOU CAN EDIT AFTER THIS LINE =========
  App.setup();
}

void loop() {
  App.loop();
}
