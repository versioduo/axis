// © Kay Sievers <kay@versioduo.com>, 2020-2024
// SPDX-License-Identifier: Apache-2.0

#include <V2BHY1.h>
#include <V2Buttons.h>
#include <V2Color.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <Wire.h>

V2DEVICE_METADATA("com.versioduo.axis", 4, "versioduo:samd:axis");

static V2LED::WS2812 LED(2, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);

static class Sensor : public V2BHY1 {
public:
  bool compass{};

  Sensor() : V2BHY1(&Wire, PIN_SENSOR_INTERRUPT){};

  // Subtract the recorded home postion to return the relative orientation to it.
  V23D::Quaternion getRotation() {
    return _setup.home * readDevice();
  }

  void setLED() {
    switch (_setup.state) {
      case State::Init:
        LED.setHSV(V2Color::Green, 0.9, 0.2);
        break;

      case State::Up:
        LED.setHSV(V2Color::Cyan, 0.9, 0.6);
        break;

      case State::Calibrated:
        LED.setHSV(V2Color::Orange, 0.9, 0.4);
        break;
    }
  }

  // Record the gravity vector when the tip of the device points up / to the
  // sky. It is not required to point to the exact up / z axis, an angle smaller
  // or larger than 90 degrees from the zero / forward positon leads to the
  // same result. Only the rotation axis between the tip and zero matters.
  void setTip() {
    _setup.up    = getGravity().normalize();
    _setup.state = State::Up;
    setLED();
  }

  // Record the current orientation. It will be substracted from future measurements
  // to use this as home / zero / start position.
  void setHome() {
    if (_setup.state == State::Up && !setForward())
      LED.splashHSV(0.3, V2Color::Red, 1, 0.6);

    else
      LED.splashHSV(0.3, V2Color::Orange, 0.9, 0.6);

    _setup.home = readDevice().getConjugate();
    setLED();
  }

  void setup(V23D::Quaternion calibration) {
    _setup = {.calibration{calibration}};
    if (!calibration.isEqual(V23D::Quaternion()))
      _setup.state = State::Calibrated;

    setLED();
  }

  V23D::Quaternion getSetup() {
    return _setup.calibration;
  }

private:
  enum class State { Init, Up, Calibrated };
  struct {
    State state{};
    V23D::Vector3 up;
    V23D::Quaternion calibration;
    V23D::Quaternion home;
  } _setup;

  // Read the absolute orientation of the sensor using the magnetometer /
  // earth's magnetic north (it might jump when the magnetic field reading
  // is disturbed or not yet known).
  // Or read the orientation without using the magentometer (uses the rotation
  // after starting up, and the orientation might drift over time).
  V23D::Quaternion readSensor() {
    return compass ? getGeoOrientation() : getOrientation();
  }

  // Apply the calibration (recorded by setTip + setForward sequence).
  V23D::Quaternion readDevice() {
    return _setup.calibration * readSensor() * _setup.calibration.getConjugate();
  }

  // Calculate the rotation needed to rotate the sensor's frame into the device's
  // frame.
  // 1. The device points up / to the sky / z axis. The button is pressed twice /
  //    double click. The LED turns cyan.
  // 2. The device points forward, the zero position. The rotation axis from up
  //    to forward defines the x axis of the device, the gravity defines the
  //    negative z axis.
  //    The button is pressed once again, the LED of the calibrated device turns
  //    orange.
  bool setForward() {
    if (_setup.state != State::Up) {
      _setup.state = State::Init;
      return false;
    }

    const V23D::Vector3 yAxis = getGravity().normalize();

    // Require a defined/significant movement / angle to prevent wrong
    // calibration values.
    const float angle = V23D::radToDeg(_setup.up.getAngleBetween(yAxis));
    if (angle < 45.f || angle > 135.f) {
      _setup.state = State::Init;
      return false;
    }

    const V23D::Vector3 xAxis = _setup.up.getCross(yAxis).normalize();

    // Calculate the calibration / rotation quaternion to move the sensor's frame
    // into the device's frame. This way the device can be mounted / held at any
    // position / any degree and set to zero with the axes properly aligned.
    _setup.calibration = V23D::Attitude::fromAccelerometerMagnetometer(yAxis, xAxis);
    _setup.state       = State::Calibrated;
    return true;
  }
} Sensor;

// Config, written to EEPROM.
static constexpr struct Configuration {
  uint8_t channel{};
  bool compass{true};

  struct {
    float w{1};
    float x{};
    float y{};
    float z{};
  } calibration;

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

    usb.ports.standard = 0;

    configuration = {.version{2}, .size{sizeof(config)}, .data{&config}};
  }

  enum class CC {
    Quaternion        = V2MIDI::CC::GeneralPurpose1,
    Home              = V2MIDI::CC::Controller14,
    SaveConfiguration = V2MIDI::CC::Controller15,
    Light             = V2MIDI::CC::Controller89,
  };

  Configuration config{ConfigurationDefault};

  // Store the tip and home orientation in the EPROM.
  void storeConfiguration() {
    config.calibration.w = Sensor.getSetup().w;
    config.calibration.x = Sensor.getSetup().x;
    config.calibration.y = Sensor.getSetup().y;
    config.calibration.z = Sensor.getSetup().z;

    writeConfiguration();
    LED.splashHSV(0.3, V2Color::Magenta, 0.8, 0.5);
  }

private:
  V2MIDI::CC::HighResolution<V2MIDI::CC::GeneralPurpose1, 4> _hires;
  uint32_t _usec{};
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
  float _lightMax{};

  void handleReset() override {
    _hires.reset();
    _lightMax   = 100.f / 127.f;
    _quaternion = {};
    _euler      = {};

    LED.reset();
    updateBrightness();
    Sensor.compass = config.compass;
    Sensor.setup({config.calibration.w, config.calibration.x, config.calibration.y, config.calibration.z});
  }

  void updateBrightness() {
    const float min   = 0.05;
    const float max   = 0.75;
    const float range = (max - min) * _lightMax;
    LED.setMaxBrightness(min + range);
    Sensor.setLED();
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
    if ((uint32_t)(V2Base::getUsec() - _usec) < 20 * 1000)
      return;

    V23D::Quaternion q = Sensor.getRotation();

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
      const uint8_t y = (e.yaw / (float)M_PI + 1.f) / 2.f * 127.f;
      if (_euler.yaw != y) {
        send(_midi.setControlChange(config.channel, config.euler.yaw, y));
        _euler.yaw = y;
      }

      const uint8_t p = (e.pitch / (float)M_PI + 1.f) / 2.f * 127.f;
      if (_euler.pitch != p) {
        send(_midi.setControlChange(config.channel, config.euler.pitch, p));
        _euler.pitch = p;
      }

      const uint8_t r = (e.roll / (float)M_PI + 1.f) / 2.f * 127.f;
      if (_euler.roll != r) {
        send(_midi.setControlChange(config.channel, config.euler.roll, r));
        _euler.roll = r;
      }
    }

    _usec = V2Base::getUsec();
  }

  bool handleSend(V2MIDI::Packet *midi) override {
    usb.midi.send(midi);
    return true;
  }

  void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) override {
    switch (controller) {
      case (uint8_t)CC::Home:
        Sensor.setHome();
        break;

      case (uint8_t)CC::SaveConfiguration:
        storeConfiguration();
        break;

      case (uint8_t)CC::Light:
        _lightMax = (float)value / 127.f;
        updateBrightness();
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
    JsonObject jsonPower  = json["sensor"].to<JsonObject>();
    jsonPower["product"]  = Sensor.getProductID();
    jsonPower["revision"] = Sensor.getRevisionID();
    jsonPower["software"] = Sensor.getRAMVersion();
  }

  void exportSettings(JsonArray json) override {
    {
      JsonObject setting = json.add<JsonObject>();
      setting["type"]    = "number";
      setting["title"]   = "MIDI";
      setting["label"]   = "Channel";
      setting["min"]     = 1;
      setting["max"]     = 16;
      setting["input"]   = "select";
      setting["path"]    = "midi/channel";
    }
    {
      JsonObject setting = json.add<JsonObject>();
      setting["type"]    = "toggle";
      setting["title"]   = "Sensor";
      setting["label"]   = "Mode";
      setting["text"]    = "Compass";
      setting["path"]    = "compass";
    }
    {
      JsonObject setting = json.add<JsonObject>();
      setting["type"]    = "toggle";
      setting["ruler"]   = true;
      setting["text"]    = "Euler";
      setting["label"]   = "Messages";
      setting["path"]    = "euler/enabled";
    }
    {
      JsonObject setting = json.add<JsonObject>();
      setting["type"]    = "controller";
      setting["label"]   = "Yaw";
      setting["path"]    = "euler/yaw";
      setting["default"] = ConfigurationDefault.euler.yaw;
    }
    {
      JsonObject setting = json.add<JsonObject>();
      setting["type"]    = "controller";
      setting["label"]   = "Pitch";
      setting["path"]    = "euler/pitch";
      setting["default"] = ConfigurationDefault.euler.pitch;
    }
    {
      JsonObject setting = json.add<JsonObject>();
      setting["type"]    = "controller";
      setting["label"]   = "Roll";
      setting["path"]    = "euler/roll";
      setting["default"] = ConfigurationDefault.euler.roll;
    }
  }

  void importConfiguration(JsonObject json) override {
    JsonObject jsonMidi = json["midi"];
    if (jsonMidi) {
      if (!jsonMidi["channel"].isNull()) {
        uint8_t channel = jsonMidi["channel"];

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

    JsonObject jsonCalibration = json["calibration"];
    if (jsonCalibration) {
      config.calibration.w = jsonCalibration["w"];
      config.calibration.x = jsonCalibration["x"];
      config.calibration.y = jsonCalibration["y"];
      config.calibration.z = jsonCalibration["z"];
    }

    JsonObject jsonEuler = json["euler"];
    if (jsonEuler) {
      if (!jsonEuler["enabled"].isNull())
        config.euler.enabled = jsonEuler["enabled"];

      config.euler.yaw   = jsonEuler["yaw"];
      config.euler.pitch = jsonEuler["pitch"];
      config.euler.roll  = jsonEuler["roll"];
    }

    reset();
  }

  void exportConfiguration(JsonObject json) override {
    json["#midi"]        = "The MIDI settings";
    JsonObject jsonMidi  = json["midi"].to<JsonObject>();
    jsonMidi["#channel"] = "The channel to send notes and control values to";
    jsonMidi["channel"]  = config.channel + 1;

    json["#compass"] = "Use the compass for absolute coordinates";
    json["compass"]  = config.compass;

    JsonObject jsonCalibration = json["calibration"].to<JsonObject>();
    jsonCalibration["#"]       = "The rotation of the sensor to the device's zero position";
    jsonCalibration["w"]       = serialized(String(config.calibration.w, 4));
    jsonCalibration["x"]       = serialized(String(config.calibration.x, 4));
    jsonCalibration["y"]       = serialized(String(config.calibration.y, 4));
    jsonCalibration["z"]       = serialized(String(config.calibration.z, 4));

    JsonObject jsonEuler = json["euler"].to<JsonObject>();
    jsonEuler["#"]       = "The controller numbers for Euler coordinates";
    jsonEuler["enabled"] = config.euler.enabled;
    jsonEuler["yaw"]     = config.euler.yaw;
    jsonEuler["pitch"]   = config.euler.pitch;
    jsonEuler["roll"]    = config.euler.roll;
  }

  void exportInput(JsonObject json) override {
    JsonArray jsonControllers = json["controllers"].to<JsonArray>();
    {
      JsonObject jsonController = jsonControllers.add<JsonObject>();
      jsonController["name"]    = "Home";
      jsonController["type"]    = "momentary";
      jsonController["number"]  = (uint8_t)CC::Home;
    }
    {
      JsonObject jsonController = jsonControllers.add<JsonObject>();
      jsonController["name"]    = "Save";
      jsonController["type"]    = "momentary";
      jsonController["number"]  = (uint8_t)CC::SaveConfiguration;
    }
    {
      JsonObject jsonController = jsonControllers.add<JsonObject>();
      jsonController["name"]    = "Brightness";
      jsonController["number"]  = (uint8_t)CC::Light;
      jsonController["value"]   = (uint8_t)(_lightMax * 127.f);
    }
  }

  void exportOutput(JsonObject json) override {
    json["channel"] = config.channel;

    JsonArray jsonControllers = json["controllers"].to<JsonArray>();
    {
      JsonObject jsonController   = jsonControllers.add<JsonObject>();
      jsonController["name"]      = "Quaternion W";
      jsonController["number"]    = (uint8_t)CC::Quaternion + 0;
      jsonController["value"]     = _hires.getMSB((uint8_t)CC::Quaternion + 0);
      jsonController["valueFine"] = _hires.getLSB((uint8_t)CC::Quaternion + 0);
    }
    {
      JsonObject jsonController   = jsonControllers.add<JsonObject>();
      jsonController["name"]      = "Quaternion X";
      jsonController["number"]    = (uint8_t)CC::Quaternion + 1;
      jsonController["value"]     = _hires.getMSB((uint8_t)CC::Quaternion + 1);
      jsonController["valueFine"] = _hires.getLSB((uint8_t)CC::Quaternion + 1);
    }
    {
      JsonObject jsonController   = jsonControllers.add<JsonObject>();
      jsonController["name"]      = "Quaternion Y";
      jsonController["number"]    = (uint8_t)CC::Quaternion + 2;
      jsonController["value"]     = _hires.getMSB((uint8_t)CC::Quaternion + 2);
      jsonController["valueFine"] = _hires.getLSB((uint8_t)CC::Quaternion + 2);
    }
    {
      JsonObject jsonController   = jsonControllers.add<JsonObject>();
      jsonController["name"]      = "Quaternion Z";
      jsonController["number"]    = (uint8_t)CC::Quaternion + 3;
      jsonController["value"]     = _hires.getMSB((uint8_t)CC::Quaternion + 3);
      jsonController["valueFine"] = _hires.getLSB((uint8_t)CC::Quaternion + 3);
    }

    if (config.euler.enabled) {
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Euler Yaw";
        jsonController["number"]  = config.euler.yaw;
        jsonController["value"]   = _euler.yaw;
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Euler Pitch";
        jsonController["number"]  = config.euler.pitch;
        jsonController["value"]   = _euler.pitch;
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Euler Roll";
        jsonController["number"]  = config.euler.roll;
        jsonController["value"]   = _euler.roll;
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

    if (_midi.getPort() == 0)
      Device.dispatch(&Device.usb.midi, &_midi);
  }

private:
  V2MIDI::Packet _midi{};
} MIDI;

static class Button : public V2Buttons::Button {
public:
  Button() : V2Buttons::Button(&_config, PIN_BUTTON) {}

private:
  const V2Buttons::Config _config{.clickUsec{200 * 1000}, .holdUsec{500 * 1000}};

  void handleHold(uint8_t count) override {
    switch (count) {
      case 1:
        Device.storeConfiguration();
        break;

      case 2:
        Device.config.calibration = {1, 0, 0, 0};
        Device.reset();
        Device.storeConfiguration();
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
  Wire.setClock(1000000);
  Wire.setTimeout(1);

  LED.begin();

  Sensor.begin();
  Button.begin();

  Device.begin();
  Device.reset();
}

void loop() {
  LED.loop();
  MIDI.loop();
  Sensor.loop();
  V2Buttons::loop();
  Device.loop();

  if (Device.idle())
    Device.sleep();
}
