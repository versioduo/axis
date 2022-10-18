// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include <V23D.h>
#include <V2BNO055.h>
#include <V2Buttons.h>
#include <V2Color.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <Wire.h>

V2DEVICE_METADATA("com.versioduo.axis", 21, "versioduo:samd:axis");

static V2LED::WS2812 LED(2, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
static V2Link::Port Plug(&SerialPlug);
static V2Link::Port Socket(&SerialSocket);

static class Sensor : public V2BNO055 {
public:
  V23D::Quaternion home;
  V23D::Quaternion tip;

  Sensor() : V2BNO055(&Wire, PIN_SENSOR_RESET, PIN_SENSOR_INTERRUPT) {}

  bool getOrientation(V23D::Quaternion &q) {
    // Get the current orientation of the sensor referencing the earth.
    V23D::Quaternion now;
    if (!readOrientation(now))
      return false;

    // Substract the current orientation from home to get the relative movement.
    q = home * now.getConjugate();
    return true;
  }

  // The absolute orientation of the sensor in relation to the earth. Used as
  // the home position to calculate the relaive orientation to.
  bool setHome() {
    if (!readOrientation(home))
      return false;

    LED.splashHSV(0.3, V2Color::Orange, 0.8, 0.4);
    return true;
  }

  // The orientation when the tip of the device points to the top (during setup).
  // It identifies the tip of the device/sensor to align the X/Y axis.
  bool setTip() {
    if (!readOrientation(tip))
      return false;

    LED.splashHSV(0.3, V2Color::Cyan, 0.8, 0.4);
    return true;
  }

private:
  void handleReset() {
    LED.splashHSV(0.3, V2Color::Blue, 0.8, 0.4);
  }

  // Query the current absolute orientation from the sensor. The sensor
  // always returns unit quaternions.
  bool readOrientation(V23D::Quaternion &q) {
    return readQuaternionData(q.w, q.x, q.y, q.z);
  }
} Sensor;

// Config, written to EEPROM
static constexpr struct Configuration {
  uint8_t channel{};
  bool compass{true};

  struct {
    float w{1};
    float x{};
    float y{};
    float z{};
  } home;

  struct {
    bool enabled{};
    uint8_t yaw{V2MIDI::CC::GeneralPurpose1 + 4};
    uint8_t pitch{V2MIDI::CC::GeneralPurpose1 + 5};
    uint8_t roll{V2MIDI::CC::GeneralPurpose1 + 6};
  } euler;
} ConfigurationDefault;

static class Device : public V2Device {
public:
  Device() : V2Device() {
    metadata.vendor      = "Versio Duo";
    metadata.product     = "V2 axis";
    metadata.description = "Orientation Sensor";
    metadata.home        = "https://versioduo.com/#axis";

    system.download  = "https://versioduo.com/download";
    system.configure = "https://versioduo.com/configure";

    configuration = {.version{1}, .size{sizeof(config)}, .data{&config}};
  }

  enum class CC {
    Quaternion = V2MIDI::CC::GeneralPurpose1,
    Home       = V2MIDI::CC::Controller14,
    SaveHome   = V2MIDI::CC::Controller15,
    Light      = V2MIDI::CC::Controller89,
  };

  Configuration config{ConfigurationDefault};

  // Store the home orientation in EEPROM.
  void storeHome() {
    Sensor.setHome();
    config.home.w = Sensor.home.w;
    config.home.x = Sensor.home.x;
    config.home.y = Sensor.home.y;
    config.home.z = Sensor.home.z;
    writeConfiguration();
    LED.splashHSV(0.3, V2Color::Magenta, 0.8, 0.4);
  }

private:
  V2MIDI::CC::HighResolution<V2MIDI::CC::GeneralPurpose1, 4> _hires;
  long _usec{};
  struct {
    uint8_t w;
    uint8_t x;
    uint8_t y;
    uint8_t z;
  } _quaternion{};

  struct {
    uint8_t yaw;
    uint8_t roll;
    uint8_t pitch;
  } _euler{};

  V2MIDI::Packet _midi{};
  float _light_max{};

  void configureSensor() {
    Sensor.reset(config.compass ? BNO055_OPERATION_MODE_NDOF : BNO055_OPERATION_MODE_IMUPLUS);
    Sensor.home = V23D::Quaternion(config.home.w, config.home.x, config.home.y, config.home.z);
  }

  void handleReset() override {
    _hires.reset();
    _light_max  = 100.f / 127.f;
    _quaternion = {};
    _euler      = {};

    LED.reset();
    configureSensor();
  }

  void allNotesOff() {
    // Send the current values when 'Stop' is pressed in audio workstation.
    _hires.send(this, config.channel, (uint8_t)CC::Quaternion + 0);
    _hires.send(this, config.channel, (uint8_t)CC::Quaternion + 1);
    _hires.send(this, config.channel, (uint8_t)CC::Quaternion + 2);
    _hires.send(this, config.channel, (uint8_t)CC::Quaternion + 3);

    if (config.euler.enabled) {
      send(_midi.setControlChange(config.channel, config.euler.yaw, _euler.yaw));
      send(_midi.setControlChange(config.channel, config.euler.pitch, _euler.pitch));
      send(_midi.setControlChange(config.channel, config.euler.roll, _euler.roll));
    }
  }

  void handleLoop() override {
    if ((unsigned long)(micros() - _usec) < 50 * 1000)
      return;

    V23D::Quaternion q;
    if (!Sensor.getOrientation(q))
      return;

    if (_hires.set((uint8_t)CC::Quaternion + 0, (q.w + 1.f) * 8191.f))
      _hires.send(this, config.channel, (uint8_t)CC::Quaternion + 0);

    if (_hires.set((uint8_t)CC::Quaternion + 1, (q.x + 1.f) * 8191.f))
      _hires.send(this, config.channel, (uint8_t)CC::Quaternion + 1);

    if (_hires.set((uint8_t)CC::Quaternion + 2, (q.y + 1.f) * 8191.f))
      _hires.send(this, config.channel, (uint8_t)CC::Quaternion + 2);

    if (_hires.set((uint8_t)CC::Quaternion + 3, (q.z + 1.f) * 8191.f))
      _hires.send(this, config.channel, (uint8_t)CC::Quaternion + 3);

    if (config.euler.enabled) {
      const V23D::Euler e = V23D::Euler::fromQuaternion(q);

      // Map the ranges to MIDI Control values.
      const uint8_t y = (e.yaw / M_PI + 1.f) / 2.f * 127.f;
      if (_euler.yaw != y) {
        send(_midi.setControlChange(config.channel, config.euler.yaw, y));
        _euler.yaw = y;
      }

      const uint8_t p = (e.pitch / M_PI + 1.f) / 2.f * 127.f;
      if (_euler.pitch != p) {
        send(_midi.setControlChange(config.channel, config.euler.pitch, p));
        _euler.pitch = p;
      }

      const uint8_t r = (e.roll / M_PI + 1.f) / 2.f * 127.f;
      if (_euler.roll != r) {
        send(_midi.setControlChange(config.channel, config.euler.roll, r));
        _euler.roll = r;
      }
    }

    _usec = micros();
  }

  bool handleSend(V2MIDI::Packet *midi) override {
    usb.midi.send(midi);
    Plug.send(midi);
    return true;
  }

  void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) override {
    switch (controller) {
      case (uint8_t)CC::Home:
        Sensor.setHome();
        break;

      case (uint8_t)CC::SaveHome:
        storeHome();
        break;

      case (uint8_t)CC::Light:
        _light_max = (float)value / 127.f;
        break;

      case V2MIDI::CC::AllSoundOff:
      case V2MIDI::CC::AllNotesOff:
        allNotesOff();
        break;
    }
  }

  void handleSystemReset() override {
    reset();
  }

  void exportSystem(JsonObject json) override {
    {
      uint16_t revision;
      if (bno055_read_sw_rev_id(&revision) == 0) {
        char s[16];
        sprintf(s, "%d.%d", revision >> 8, revision & 0xff);
        json["revision"] = s;
      }
    }
    {
      float t;
      if (Sensor.readTemperature(t))
        json["temperature"] = serialized(String(t, 1));
    }

    {
      JsonObject json_calibration = json.createNestedObject("calibration");
      uint8_t status;
      if (bno055_get_mag_calib_stat(&status) == 0)
        json_calibration["compass"] = status == 3;
    }
  }

  void exportSettings(JsonArray json) override {
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "number";
      setting["title"]   = "MIDI";
      setting["label"]   = "Channel";
      setting["min"]     = 1;
      setting["max"]     = 16;
      setting["input"]   = "select";
      setting["path"]    = "midi/channel";
    }
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "toggle";
      setting["title"]   = "Sensor";
      setting["label"]   = "Mode";
      setting["text"]    = "Compass";
      setting["path"]    = "compass";
    }

    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "toggle";
      setting["ruler"]   = true;
      setting["text"]    = "Euler";
      setting["label"]   = "Messages";
      setting["path"]    = "euler/enabled";
    }
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "controller";
      setting["label"]   = "Yaw";
      setting["path"]    = "euler/yaw";
      setting["default"] = ConfigurationDefault.euler.yaw;
    }
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "controller";
      setting["label"]   = "Pitch";
      setting["path"]    = "euler/pitch";
      setting["default"] = ConfigurationDefault.euler.pitch;
    }
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "controller";
      setting["label"]   = "Roll";
      setting["path"]    = "euler/roll";
      setting["default"] = ConfigurationDefault.euler.roll;
    }
  }

  void importConfiguration(JsonObject json) override {
    JsonObject json_midi = json["midi"];
    if (json_midi) {
      if (!json_midi["channel"].isNull()) {
        uint8_t channel = json_midi["channel"];

        if (channel < 1)
          config.channel = 0;
        else if (channel > 16)
          config.channel = 15;
        else
          config.channel = channel - 1;
      }
    }

    if (!json["compass"].isNull())
      config.compass = json["compass"];

    JsonObject json_home = json["home"];
    if (json_home) {
      config.home.w = json_home["w"];
      config.home.x = json_home["x"];
      config.home.y = json_home["y"];
      config.home.z = json_home["z"];
    }

    JsonObject json_euler = json["euler"];
    if (json_euler) {
      if (!json_euler["enabled"].isNull())
        config.euler.enabled = json_euler["enabled"];

      config.euler.yaw   = json_euler["yaw"];
      config.euler.pitch = json_euler["pitch"];
      config.euler.roll  = json_euler["roll"];
    }

    configureSensor();
  }

  void exportConfiguration(JsonObject json) override {
    json["#midi"]         = "The MIDI settings";
    JsonObject json_midi  = json.createNestedObject("midi");
    json_midi["#channel"] = "The channel to send notes and control values to";
    json_midi["channel"]  = config.channel + 1;

    json["#compass"] = "Use the compass for absolute coordinates";
    json["compass"]  = config.compass;

    JsonObject json_home = json.createNestedObject("home");
    json_home["#"]       = "The home / zero position";
    json_home["w"]       = serialized(String(config.home.w, 4));
    json_home["x"]       = serialized(String(config.home.x, 4));
    json_home["y"]       = serialized(String(config.home.y, 4));
    json_home["z"]       = serialized(String(config.home.z, 4));

    JsonObject json_euler = json.createNestedObject("euler");
    json_euler["#"]       = "The controller numbers for Euler coordinates";
    json_euler["enabled"] = config.euler.enabled;
    json_euler["yaw"]     = config.euler.yaw;
    json_euler["pitch"]   = config.euler.pitch;
    json_euler["roll"]    = config.euler.roll;
  }

  void exportInput(JsonObject json) override {
    JsonArray json_controllers = json.createNestedArray("controllers");
    {
      JsonObject json_controller = json_controllers.createNestedObject();
      json_controller["name"]    = "Home";
      json_controller["type"]    = "momentary";
      json_controller["number"]  = (uint8_t)CC::Home;
    }
    {
      JsonObject json_controller = json_controllers.createNestedObject();
      json_controller["name"]    = "Save Home";
      json_controller["type"]    = "momentary";
      json_controller["number"]  = (uint8_t)CC::SaveHome;
    }
    {
      JsonObject json_controller = json_controllers.createNestedObject();
      json_controller["name"]    = "Brightness";
      json_controller["number"]  = (uint8_t)CC::Light;
      json_controller["value"]   = (uint8_t)(_light_max * 127.f);
    }
  }

  void exportOutput(JsonObject json) override {
    json["channel"] = config.channel;

    JsonArray json_controllers = json.createNestedArray("controllers");
    {
      JsonObject json_controller   = json_controllers.createNestedObject();
      json_controller["name"]      = "Quaternion W";
      json_controller["number"]    = (uint8_t)CC::Quaternion + 0;
      json_controller["value"]     = _hires.getMSB((uint8_t)CC::Quaternion + 0);
      json_controller["valueFine"] = _hires.getLSB((uint8_t)CC::Quaternion + 0);
    }
    {
      JsonObject json_controller   = json_controllers.createNestedObject();
      json_controller["name"]      = "Quaternion X";
      json_controller["number"]    = (uint8_t)CC::Quaternion + 1;
      json_controller["value"]     = _hires.getMSB((uint8_t)CC::Quaternion + 1);
      json_controller["valueFine"] = _hires.getLSB((uint8_t)CC::Quaternion + 1);
    }
    {
      JsonObject json_controller   = json_controllers.createNestedObject();
      json_controller["name"]      = "Quaternion Y";
      json_controller["number"]    = (uint8_t)CC::Quaternion + 2;
      json_controller["value"]     = _hires.getMSB((uint8_t)CC::Quaternion + 2);
      json_controller["valueFine"] = _hires.getLSB((uint8_t)CC::Quaternion + 2);
    }
    {
      JsonObject json_controller   = json_controllers.createNestedObject();
      json_controller["name"]      = "Quaternion Z";
      json_controller["number"]    = (uint8_t)CC::Quaternion + 3;
      json_controller["value"]     = _hires.getMSB((uint8_t)CC::Quaternion + 3);
      json_controller["valueFine"] = _hires.getLSB((uint8_t)CC::Quaternion + 3);
    }

    if (config.euler.enabled) {
      {
        JsonObject json_controller = json_controllers.createNestedObject();
        json_controller["name"]    = "Euler Yaw";
        json_controller["number"]  = config.euler.yaw;
        json_controller["value"]   = _euler.yaw;
      }
      {
        JsonObject json_controller = json_controllers.createNestedObject();
        json_controller["name"]    = "Euler Pitch";
        json_controller["number"]  = config.euler.pitch;
        json_controller["value"]   = _euler.pitch;
      }
      {
        JsonObject json_controller = json_controllers.createNestedObject();
        json_controller["name"]    = "Euler Roll";
        json_controller["number"]  = config.euler.roll;
        json_controller["value"]   = _euler.roll;
      }
    }
  }
} Device;

// Dispatch MIDI packets
static class MIDI {
public:
  void loop() {
    if (!Device.usb.midi.receive(&_midi))
      return;

    if (_midi.getPort() == 0) {
      Device.dispatch(&Device.usb.midi, &_midi);

    } else {
      _midi.setPort(_midi.getPort() - 1);
      Socket.send(&_midi);
    }
  }

private:
  V2MIDI::Packet _midi{};
} MIDI;

// Dispatch Link packets
static class Link : public V2Link {
public:
  Link() : V2Link(&Plug, &Socket) {}

private:
  V2MIDI::Packet _midi{};

  // Receive a host event from our parent device
  void receivePlug(V2Link::Packet *packet) override {
    if (packet->getType() == V2Link::Packet::Type::MIDI) {
      packet->receive(&_midi);
      Device.dispatch(&Plug, &_midi);
    }
  }

  // Forward children device events to the host
  void receiveSocket(V2Link::Packet *packet) override {
    if (packet->getType() == V2Link::Packet::Type::MIDI) {
      uint8_t address = packet->getAddress();
      if (address == 0x0f)
        return;

      if (Device.usb.midi.connected()) {
        packet->receive(&_midi);
        _midi.setPort(address + 1);
        Device.usb.midi.send(&_midi);
      }
    }
  }
} Link;

static class Button : public V2Buttons::Button {
public:
  Button() : V2Buttons::Button(&_config, PIN_BUTTON) {}

private:
  const V2Buttons::Config _config{.click_usec{150 * 1000}, .hold_usec{350 * 1000}};

  void handleHold(uint8_t count) override {
    switch (count) {
      case 1:
        Device.storeHome();
        break;
    }
  }

  void handleClick(uint8_t count) override {
    switch (count) {
      case 0:
        Sensor.setHome();
        break;

      case 1:
        Sensor.setTip();
        break;
    }
  }
} Button;

void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(1);

  LED.begin();
  LED.setMaxBrightness(0.5);

  Plug.begin();
  Socket.begin();

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. This needs to be after begin().
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 2);

  Sensor.begin();
  Button.begin();

  Device.link = &Link;
  Device.begin();
  Device.reset();
}

void loop() {
  LED.loop();
  MIDI.loop();
  Link.loop();
  Sensor.loop();
  V2Buttons::loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
