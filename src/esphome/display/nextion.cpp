#include "esphome/defines.h"

#ifdef USE_NEXTION

#include "esphome/display/nextion.h"
#include "esphome/log.h"

ESPHOME_NAMESPACE_BEGIN

namespace display {

static const char *TAG = "display.nextion";

void Nextion::setup() {
  this->send_command_("");
  this->ack_();
  this->goto_page("0");
}
float Nextion::get_setup_priority() const {
  return setup_priority::POST_HARDWARE;
}
void Nextion::update() {
  if (this->writer_.has_value()) {
    (*this->writer_)(*this);
  }
}
void Nextion::send_command_(const char *command) {
  // Flush RX...
  this->loop();

  this->write_str(command);
  const uint8_t data[3] = {0xFF, 0xFF, 0xFF};
  this->write_array(data, sizeof(data));
}

bool Nextion::ack_() {
  uint8_t bytes[4] = {0x00, };
  if (!this->read_array(bytes, 4)) {
    ESP_LOGW(TAG, "Nextion returned no ACK data!");
    return false;
  }

  if (bytes[1] != 0xFF || bytes[2] != 0xFF || bytes[3] != 0xFF) {
    ESP_LOGW(TAG, "Nextion returned invalid ACK data!");
    return false;
  }

  switch (bytes[0]) {
    case 0x01: // successful execution of instruction
      return true;
    case 0x00: // invalid instruction
    case 0x02: // component ID invalid
    case 0x03: // page ID invalid
    case 0x04: // picture ID invalid
    case 0x05: // font ID invalid
    case 0x11: // baud rate setting invalid
    case 0x12: // curve control ID number or channel number is invalid
    case 0x1A: // variable name invalid
    case 0x1B: // variable operation invalid
    case 0x1C: // failed to assign
    case 0x1D: // operate EEPROM failed
    case 0x1E: // parameter quantity invalid
    case 0x1F: // IO operation failed
    case 0x20: // undefined escape characters
    case 0x23: // too long variable name
    default:
      ESP_LOGW(TAG, "Nextion returned NACK with code 0x%02X", bytes[0]);
      return false;
  }
}
void Nextion::set_component_text(const char *component, const char *text) {
  this->send_command_printf_("%s.txt=\"%s\"", component, text);
}
void Nextion::set_component_value(const char *component, int value) {
  this->send_command_printf_("%s.val=%d", component, value);
}
void Nextion::set_component_background_color(const char *component, const char *color) {
  this->send_command_printf_("%s.bco=\"%s\"", component, color);
}
void Nextion::set_component_pressed_background_color(const char *component, const char *color) {
  this->send_command_printf_("%s.bco2=\"%s\"", component, color);
}
void Nextion::set_component_font_color(const char *component, const char *color) {
  this->send_command_printf_("%s.pco=\"%s\"", component, color);
}
void Nextion::set_component_pressed_font_color(const char *component, const char *color) {
  this->send_command_printf_("%s.pco2=\"%s\"", component, color);
}
void Nextion::set_component_coordinates(const char *component, int x, int y) {
  this->send_command_printf_("%s.xcen=%d", component, x);
  this->send_command_printf_("%s.ycen=%d", component, y);
}
void Nextion::set_component_font(const char *component, uint8_t font_id) {
  this->send_command_printf_("%s.font=%d", component, font_id);
}
void Nextion::goto_page(const char *page) {
  this->send_command_printf_("page %s", page);
}
bool Nextion::send_command_printf_(const char *format, ...) {
  char buffer[256];
  va_list arg;
  va_start(arg, format);
  int ret = vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);
  if (ret <= 0) {
    ESP_LOGW(TAG, "Building command for format '%s' failed!", format);
    return false;
  }
  this->send_command_(buffer);
  if (!this->ack_()) {
    ESP_LOGW(TAG, "Sending command '%s' failed because no ACK was received", buffer);
    return false;
  }

  return true;
}
void Nextion::hide_component(const char *component) {
  this->send_command_printf_("vis %s,0", component);
}
void Nextion::show_component(const char *component) {
  this->send_command_printf_("vis %s,1", component);
}
void Nextion::enable_component_touch(const char *component) {
  this->send_command_printf_("tsw %s,1", component);
}
void Nextion::disable_component_touch(const char *component) {
  this->send_command_printf_("tsw %s,0", component);
}
void Nextion::add_waveform_data(int component_id, uint8_t channel_number, uint8_t value) {
  this->send_command_printf_("add %d,%u,%u", component_id, channel_number, value);
}
void Nextion::display_picture(int picture_id, int x_start, int y_start) {
  this->send_command_printf_("pic %d,%d,%d", x_start, y_start, picture_id);
}
void Nextion::fill_area(int x1, int y1, int width, int height, const char *color) {
  this->send_command_printf_("fill %d,%d,%d,%d,%s", x1, y1, width, height, color);
}
void Nextion::line(int x1, int y1, int x2, int y2, const char *color) {
  this->send_command_printf_("line %d,%d,%d,%d,%s", x1, y1, x2, y2, color);
}
void Nextion::rectangle(int x1, int y1, int width, int height, const char *color) {
  this->send_command_printf_("draw %d,%d,%d,%d,%s", x1, y1, x1 + width, y1 + height, color);
}
void Nextion::circle(int center_x, int center_y, int radius, const char *color) {
  this->send_command_printf_("cir %d,%d,%d,%s", center_x, center_y, radius, color);
}
void Nextion::filled_circle(int center_x, int center_y, int radius, const char *color) {
  this->send_command_printf_("cirs %d,%d,%d,%s", center_x, center_y, radius, color);
}
void Nextion::loop() {
  while (this->available() >= 4) {
    // flush preceding filler bytes
    uint8_t temp;
    while (this->available() && this->peek_byte(&temp) && temp == 0xFF)
      this->read_byte(&temp);

    if (!this->available())
      break;

    uint8_t event;
    // event type
    this->read_byte(&event);

    uint8_t data[255];
    // total length of data (including end bytes)
    uint8_t data_length = 0;
    // message is terminated by three consecutive 0xFF
    // this variable keeps track of ohow many of those have
    // been received
    uint8_t end_length = 0;
    while (this->available() && end_length < 3 && data_length < sizeof(data)) {
      uint8_t byte;
      this->read_byte(&byte);
      if (byte == 0xFF) {
        end_length++;
      } else {
        end_length = 0;
      }
      data[data_length++] = byte;
    }

    if (end_length != 3) {
      ESP_LOGW(TAG, "Received unknown filler end bytes from Nextion!");
      continue;
    }

    data_length -= 3; // remove filler bytes

    bool invalid_data_length = false;
    switch (event) {
      case 0x65: { // touch event return data
        if (data_length != 3) {
          invalid_data_length = true;
          break;
        }
        uint8_t page_id = data[0];
        uint8_t component_id = data[1];
        uint8_t touch_event = data[2]; // 0 -> release, 1 -> press
        ESP_LOGD(TAG, "Got touch page=%u component=%u type=%s", page_id, component_id, touch_event ? "PRESS" : "RELEASE");
        for (auto *touch : this->touch_) {
          touch->process(page_id, component_id, touch_event);
        }
        break;
      }
      case 0x67:
      case 0x68: { // touch coordinate data
        if (data_length != 5) {
          invalid_data_length = true;
          break;
        }
        uint16_t x = (uint16_t(data[0]) << 8) | data[1];
        uint16_t y = (uint16_t(data[2]) << 8) | data[3];
        uint8_t touch_event = data[4]; // 0 -> release, 1 -> press
        ESP_LOGD(TAG, "Got touch at x=%u y=%u type=%s", x, y, touch_event ? "PRESS" : "RELEASE");
        break;
      }
      case 0x66: // sendme page id
      case 0x70: // string variable data return
      case 0x71: // numeric variable data return
      case 0x86: // device automatically enters into sleep mode
      case 0x87: // device automatically wakes up
      case 0x88: // system successful start up
      case 0x89: // start SD card upgrade
      case 0xFD: // data transparent transmit finished
      case 0xFE: // data transparent transmit ready
        break;
      default:
        ESP_LOGW(TAG, "Received unknown event from nextion: 0x%02X", event);
        break;
    }
    if (invalid_data_length) {
      ESP_LOGW(TAG, "Invalid data length from nextion!");
    }
  }
}
#ifdef USE_TIME
void Nextion::set_nextion_rtc_time(time::ESPTime time) {
  this->send_command_printf_("rtc0=%u", time.year);
  this->send_command_printf_("rtc1=%u", time.month);
  this->send_command_printf_("rtc2=%u", time.day_of_month);
  this->send_command_printf_("rtc3=%u", time.hour);
  this->send_command_printf_("rtc4=%u", time.minute);
  this->send_command_printf_("rtc5=%u", time.second);
}
#endif

NextionTouchComponent *Nextion::make_touch_component(const std::string &name, uint8_t page_id, uint8_t component_id) {
  auto *ret = new NextionTouchComponent(name, page_id, component_id);
  this->touch_.push_back(ret);
  return ret;
}
Nextion::Nextion(UARTComponent *parent, uint32_t update_interval)
    : PollingComponent(update_interval), UARTDevice(parent) {

}
void Nextion::set_writer(const nextion_writer_t &writer) {
  this->writer_ = writer;
}
void Nextion::set_component_text_printf(const char *component, const char *format, ...) {
  va_list arg;
  va_start(arg, format);
  char buffer[256];
  int ret = vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);
  if (ret > 0)
    this->set_component_text(component, buffer);
}

void NextionTouchComponent::process(uint8_t page_id, uint8_t component_id, bool on) {
  if (this->page_id_ == page_id && this->component_id_ == component_id) {
    this->publish_state(on);
  }
}
NextionTouchComponent::NextionTouchComponent(const std::string &name, uint8_t page_id, uint8_t component_id)
    : BinarySensor(name), page_id_(page_id), component_id_(component_id) {}

} // namespace display

ESPHOME_NAMESPACE_END

#endif //USE_NEXTION
