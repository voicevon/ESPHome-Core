// Auto generated code by esphomeyaml
// ========== AUTO GENERATED INCLUDE BLOCK BEGIN ===========
#include "esphomelib/application.h"
using namespace esphomelib;


switch_::GPIOSwitch* motor_pa;
switch_::GPIOSwitch* motor_pb;
sensor::PulseCounterSensorComponent* total_weight;
sensor::MQTTSubscribeSensor* to_feed;   // to be feeded this time.
int last_total = 0;
DeepSleepComponent *sleep;

void setup_apps(){
  //sensor_mqtt_subcribe create, register
  to_feed = App.make_mqtt_subscribe_sensor("max_weight", "feeder/config/max_weight/command");
  
  // //binary_sensor::gpio   create,register,
  motor_pa = App.make_gpio_switch("motor_pa", 5);
  motor_pb = App.make_gpio_switch("motor_pb", 4);

  //sensor::pulse_counter create,register,set up,
  total_weight = App.make_pulse_counter_sensor("weight",GPIOInputPin(14, INPUT_PULLUP));

}
void setup_servers(){
  App.set_name("feeder");
  App.set_compilation_datetime(__DATE__ ", " __TIME__);
  LogComponent *logcomponent = App.init_log(115200);
  logcomponent->set_global_log_level(ESPHOMELIB_LOG_LEVEL_DEBUG);

  WiFiComponent *wificomponent = App.init_wifi();
  WiFiAP wifiap = WiFiAP();
  wifiap.set_ssid("FuckGFW");
  wifiap.set_password("refuckgfw");
  wifiap.set_manual_ip(ManualIP{
      .static_ip = IPAddress(192, 168, 123, 198),
      .gateway = IPAddress(192, 168, 123, 1),
      .subnet = IPAddress(255, 255, 255, 0),
      .dns1 = IPAddress(192, 168, 123, 1),
      .dns2 = IPAddress(0, 0, 0, 0),
  });

  wificomponent->add_sta(wifiap);
  // OTAComponent *otacomponent = App.init_ota();
  // otacomponent->set_auth_password("1234567890");
  // otacomponent->start_safe_mode();
  mqtt::MQTTClientComponent *mqtt_mqttclientcomponent = App.init_mqtt("voicevon.vicp.io", 1883, "von", "von1970");
  // api::APIServer *api_apiserver = App.init_api_server();
  // api_apiserver->set_password("1234567890");

  sleep = App.make_deep_sleep_component();
  sleep->set_sleep_duration(60000);
  sleep->set_run_duration(20000);
}

void setup() {
  setup_servers();
  setup_apps();
  App.setup(); 
}
// ========== AUTO GENERATED INCLUDE BLOCK END ==========="


void automation_1(){
  static bool last = false;
  bool next  = false;
  total_weight->update();
  if(total_weight->state < to_feed->state + last_total)
    next = true;
  if(motor_pa->state != next)
  {
    sleep->prevent_deep_sleep();
    motor_pa->toggle();
    if( last && !next)
    {
      // weight->state = 0;  //read only
      last_total = total_weight->state;
      to_feed->state = 0;
    }
    last = next;
  }
}

void loop() {
  delay(16);
  App.loop();
  automation_1();
}
