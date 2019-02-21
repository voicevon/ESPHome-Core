// Auto generated code by esphomeyaml
// ========== AUTO GENERATED INCLUDE BLOCK BEGIN ===========
#include "esphomelib/application.h"
using namespace esphomelib;

sensor::HX711Sensor* weight;
sensor::ADCSensorComponent* tds;
sensor::ADCSensorComponent* adc_leaking;
sensor::ADCSensorComponent* adc_ap;
sensor::ADCSensorComponent* adc_sp;    //源水压力
sensor::ADCSensorComponent* adc_bp;    //净水压力

sensor::PulseCounterSensorComponent* pc_a;
sensor::PulseCounterSensorComponent* pc_b;
sensor::PulseCounterSensorComponent* pc_c;

// binary_sensor::GPIOBinarySensorComponent*  bs_sp;
// binary_sensor::GPIOBinarySensorComponent* bs_bp;
binary_sensor::GPIOBinarySensorComponent* bs_door;

binary_sensor::TemplateBinarySensor * leaking;

switch_::GPIOSwitch* r1;
switch_::GPIOSwitch* r2;
switch_::GPIOSwitch* r3;
switch_::GPIOSwitch* r4;
switch_::GPIOSwitch* r5;
switch_::GPIOSwitch* r6;

// next version:  a component named weighing_scale_hx711
// action_1 :=  reset zero mass value (long new_value)
// action_2 :=  reset second mass and value (float second_mess, long new_value) , will change the g_slope
float g_value_of_zero_mass = 8388608.0f;
float g_slope = 0.000123;
// result = (value - g_value_of_zero_mass) * slope


void setup_apps_ondebug(){
  //sensor::HX711
  weight = App.make_hx711_sensor("weight", 22, 21, 20000);
  weight->set_gain(sensor::HX711_GAIN_128);
  weight->set_unit_of_measurement("Kg");
  weight->set_filters({
      new sensor::LambdaFilter([=](float x) {
        auto first_mass = 0.0; // first known mass was 0kg
        auto first_value = g_value_of_zero_mass; // value for the first known mass was 120
        auto second_mass = 1000.0; // second mass was 1kg
        auto second_value = 8588609; // second value was 810
        return map(x, first_value, second_value, first_mass, second_mass);
    }),
  });
}

void setup3_gpioPin(){
  
  //sensors:  Dallas
  // sensor::DallasComponent *sensor_dallascomponent = App.make_dallas_component(19);
  // sensor::DallasTemperatureSensor *sensor_dallastemperaturesensor = sensor_dallascomponent->get_sensor_by_address("temp", 0x02021312A315AA28);
  // sensor::MQTTSensorComponent *sensor_mqttsensorcomponent = App.register_sensor(sensor_dallastemperaturesensor);



  //sensors: Adc
  tds = App.make_adc_sensor("tds", 39, 20000);
  tds->set_attenuation(ADC_11db);
  tds->set_filters({
      new sensor::SlidingWindowMovingAverageFilter(5, 5),
      new sensor::LambdaFilter([=](float x) -> optional<float> {
        float x2 = 0.54f;
        float y2 = 371.0f;
        float x1 = 0.08f;
        float y1 = 124.0f;
        float s = (y2 - y1) / (x2 - x1);
        float c = y1 - s * x1;
        return x * s + c;
    }),
  });
  tds->set_unit_of_measurement("ppm");

  adc_leaking = App.make_adc_sensor("adc_leaking", 36, 20000);
  adc_leaking->set_attenuation(ADC_11db);
  adc_leaking->set_filters({});
  adc_leaking->set_unit_of_measurement("V");

  adc_ap = App.make_adc_sensor("ap", 33, 20000);
  adc_ap->set_attenuation(ADC_11db);
  adc_ap->set_filters({
      new sensor::SlidingWindowMovingAverageFilter(5, 5),
      new sensor::LambdaFilter([=](float x) -> optional<float> {
        float x2 = 3.8f;
        float y2 = 8.1f;
        float x1 = 0.38f;
        float y1 = 0.0f;
        float s = (y2 - y1) / (x2 - x1);
        float c = y1 - s * x1;
        return x * s + c;
    }),
  });

  adc_bp = App.make_adc_sensor("bp", 34, 6000);
  adc_bp->set_attenuation(ADC_11db);
    adc_bp->set_filters({
      new sensor::SlidingWindowMovingAverageFilter(5, 5),
      new sensor::LambdaFilter([=](float x) -> optional<float> {
        float x2 = 3.8f;
        float y2 = 8.1f;
        float x1 = 0.38f;
        float y1 = 0.0f;
        float s = (y2 - y1) / (x2 - x1);
        float c = y1 - s * x1;
        return x * s + c;
    }),
  });

  adc_sp = App.make_adc_sensor("sp", 35, 20000);
  adc_sp->set_attenuation(ADC_11db);
  adc_sp->set_filters({
      new sensor::SlidingWindowMovingAverageFilter(5, 5),
      new sensor::LambdaFilter([=](float x) -> optional<float> {
        float x2 = 3.8f;
        float y2 = 8.1f;
        float x1 = 0.38f;
        float y1 = 0.0f;
        float s = (y2 - y1) / (x2 - x1);
        float c = y1 - s * x1;
        return x * s + c;
    }),
  });

  //sensors: Pulse_counterds
  pc_a = App.make_pulse_counter_sensor("pc_a", 18, 20000);
  pc_a->set_filters({
      new sensor::MultiplyFilter(0.0004854f),
  });

  pc_b = App.make_pulse_counter_sensor("pc_b", 17, 20000);
  pc_b->set_filters({
      new sensor::MultiplyFilter(0.0004854f),
  });

  pc_c = App.make_pulse_counter_sensor("pc_c", 16, 20000);
  pc_c->set_filters({
      new sensor::MultiplyFilter(0.0004854f),
  });

  //binary_sensors: gpio
  // Application::MakeGPIOBinarySensor sensor_gpio_binary_sp = App.make_gpio_binary_sensor("sp", 35);
  // bs_sp = sensor_gpio_binary_sp.gpio;

  // Application::MakeGPIOBinarySensor sensor_gpio_binary_bp = App.make_gpio_binary_sensor("bp", 34);
  // bs_bp = sensor_gpio_binary_bp.gpio;

  bs_door = App.make_gpio_binary_sensor("door", GPIOInputPin(23, INPUT_PULLUP));


  //switch.gpio
  r1 = App.make_gpio_switch("r1", 13);
  r2 = App.make_gpio_switch("r2", 32);
  r3 = App.make_gpio_switch("r3", 4);
  r4 = App.make_gpio_switch("r4", 25);
  r5 = App.make_gpio_switch("r5", 27);
  r6 = App.make_gpio_switch("r6", 14);
}
void setup4_template(){
  //template::binary_sensor 
  leaking = App.make_template_binary_sensor("leaking");
  leaking->set_template([=]() -> optional<bool> {
      if (adc_leaking->state < 0.7)
        return true;
      else
        return false;
  });
}

void setup5_display(){
  //SPI and max7219 display
  SPIComponent *spicomponent = App.init_spi(5);
  spicomponent->set_mosi(2);
  display::MAX7219Component *display_max7219 = App.make_max7219(spicomponent, 15);
  display_max7219->set_num_chips(1);
  display_max7219->set_writer([=](display::MAX7219Component & it) {
      it.print("01234567");    
  });
}

void setup1_servers(){
  App.set_compilation_datetime(__DATE__ ", " __TIME__);
  App.init_log(115200);
  
 
  WiFiComponent *wificomponent = App.init_wifi();
  WiFiAP wifiap = WiFiAP();
  wifiap.set_ssid("yuxiu");
  wifiap.set_password("yuxiu2010");
  wificomponent->add_sta(wifiap);

  // OTAComponent *otacomponent = App.init_ota();
  // otacomponent->set_auth_password("1234567890");
  // otacomponent->start_safe_mode();

  // api::APIServer *api_apiserver = App.init_api_server();
  // api_apiserver->set_password("1234567890");

  mqtt::MQTTClientComponent *mqtt_mqttclientcomponent = App.init_mqtt("voicevon.vicp.io", 1883, "von", "von1970");
  mqtt_mqttclientcomponent->set_birth_message(mqtt::MQTTMessage{
      .topic = "yuxiu/status",
      .payload = "online",
      .qos = 0,
      .retain = true,
  });
  mqtt_mqttclientcomponent->set_last_will(mqtt::MQTTMessage{
      .topic = "yuxiu/status",
      .payload = "offline",
      .qos = 0,
      .retain = true,
  });
}

void setup2_common(){
  //Status_led
  StatusLEDComponent *status_led = App.make_status_led(12);
  esphomelib::sensor::WiFiSignalSensor* sensor_wifisignalsensor = App.make_wifi_signal_sensor("wifi_signal", 60000);
  sensor_wifisignalsensor->set_filters({
      new sensor::SlidingWindowMovingAverageFilter(5, 5),
  });

  sensor::UptimeSensor *sensor_uptimesensor = App.make_uptime_sensor("Uptime");
}

void turn_off_relays()
{
  r1->turn_off();
  r2->turn_off();
  r3->turn_off();
  r4->turn_off();
  r5->turn_off();
  r6->turn_off();  
}
void setup() {
  App.set_name("yuxiu");
  setup1_servers();
  setup2_common();
  setup3_gpioPin();
  setup4_template();
  // setup5_display();
  setup_apps_ondebug();
  turn_off_relays();
  App.setup();
  turn_off_relays();
}


int working_state =0;
//  1 开启， 进水电磁阀已开。   制水中 = True
//  2 废水电磁阀已开
//  3 Pump_A已开
//  4 Pumb_B已开.  
//  10 废水电磁阀已关。   检查净水压力。  
//  11 [净水压力足够大]，废水电磁阀已开，
//  12 进水电磁阀已关，
//  13 Pump_A已关
//  14 PumP_B已关
//  0 废水电磁阀已关。  关闭过程结束。 制水中 = false
//  99 waring.
void automation_main(){

  if( adc_sp->state < 0.5f)
    working_state = 99;
  if( adc_ap->state > 9.0f)
    working_state = 99;
  if( leaking->state )
    working_state = 99;

  switch (working_state )
  {
    case 99:
      turn_off_relays();
      break;
    case 0:
      if(adc_bp->state < 2){
        r3->turn_on();
        r4->turn_off();
        r1->turn_off();
        r2->turn_off();
        delay(3000);
        working_state = 1;
      }
      break;
    case 1:
      r4->turn_on();
      delay(2000);
      working_state = 2;
      break;
    case 2:
      r1->turn_on();
      delay(2000);
      working_state = 3;
      break;
    case 3:
      r2->turn_on();
      delay(2000);
      working_state =4;
      break;
    case 4:
      r4->turn_off();
      working_state = 10;
    case 10:
      if(adc_bp->state >3.0f){
        r4->turn_on();
        delay(2000);
        working_state = 11;
      }
      break;
    case 11:
      r3->turn_off();
      delay(2000);
      working_state = 12;
      break;
    case 12:
      r1->turn_off();
      delay(2000);
      working_state = 13;
      break;
    case 13:
      r2->turn_off();
      delay(2000);
      working_state = 14;
      break;
    case 14:
      r4->turn_off();
      working_state = 0;
      break;
    default:
      break;
  }
}


void loop() {
  App.loop();
  
  automation_main();
}