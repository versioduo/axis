#include <V2BHY1.h>
#include <V2Buttons.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <Wire.h>

V2DEVICE_METADATA("com.versioduo.axis", 5, "versioduo:samd:axis");

static V2LED::WS2812 LED(2, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);

static class Sensor : public V2BHY1 {
public:
  Sensor() : V2BHY1(&Wire, PIN_SENSOR_INTERRUPT) {}

  // Subtract the recorded home postion to return the relative orientation to it.
  auto getRotation() -> V23D::Quaternion {
    return _setup.home * readCalibrated();
  }

  auto getAcceleration() -> V23D::Vector3 {
    return getGyroscope();
  }

  void updateLED() {
    switch (_setup.state) {
      case State::Init:
        LED.setHSV(V2Colour::Cyan, 0.9, 0.2);
        break;

      case State::Up:
        LED.setHSV(V2Colour::Green, 0.9, 0.6);
        break;

      case State::Calibrated:
        LED.setHSV(V2Colour::Orange, 0.9, 0.4);
        break;
    }
  }

  // The device points upwards. It is not required to point to the exact Z axis, an
  // angle smaller or larger than 90 degrees from the zero / Forward positon leads
  // to the same result. Only the rotation axis between the Tip and Forward matters.
  auto tip() {
    _setup = {
      .state = State::Up,
      .up    = getGravity().normalize(),
    };
    updateLED();
  }
  // The device points forward, the zero position. The rotation axis from Up
  // to Forward defines the Y (Pitch) axis of the device, the gravity defines
  // the Z (Yaw) axis. The button is pressed once again, the LED of the
  // now calibrated device turns orange.
  auto forward() -> bool {
    auto zAxis{getGravity().normalize()};

    // Require a defined / significant movement / angle to prevent wrong
    // calibration values.
    if (auto angle{V23D::radToDeg(_setup.up.angleBetween(zAxis))}; angle < 45.f || angle > 135.f) {
      _setup.state = State::Init;
      LED.splashHSV(0.3, V2Colour::Red, 1, 0.6);
      updateLED();
      return false;
    }

    auto yAxis{_setup.up.cross(zAxis).normalize()};

    // Calculate the calibration / rotation quaternion to move the sensor's frame
    // into the device's frame.
    _setup.calibration = V23D::Attitude::accelerometerMagnetometer(zAxis, yAxis);
    _setup.state       = State::Calibrated;

    updateLED();
    return true;
  }

  // Record the current orientation. It will be substracted from future measurements
  // to use this as home / zero / start position.
  auto home() {
    if (_setup.state == State::Up)
      forward();

    _setup.home = readCalibrated().conjugate();
  }

  auto setup(bool compass, const V23D::Quaternion& calibration) {
    _compass = compass;

    _setup = {.calibration{calibration}};
    if (!calibration.equal(V23D::Quaternion()))
      _setup.state = State::Calibrated;

    home();
    updateLED();
  }

  auto calibration() const -> const V23D::Quaternion& {
    return _setup.calibration;
  }

private:
  enum class State { Init, Up, Calibrated };
  bool _compass{};
  struct {
    State            state{};
    V23D::Vector3    up;
    V23D::Quaternion calibration;
    V23D::Quaternion home;
  } _setup;

  // Read the absolute orientation of the sensor using the magnetometer /
  // earth's magnetic north (it might jump when the magnetic field reading
  // is disturbed or not yet known).
  // Or read the orientation without using the magentometer (uses the rotation
  // after starting up, and the orientation might drift over time).
  auto read() -> V23D::Quaternion {
    return _compass ? getGeoOrientation() : getOrientation();
  }

  // Apply the calibration recorded by tip + forward sequence.
  auto readCalibrated() -> V23D::Quaternion {
    return _setup.calibration * read() * _setup.calibration.conjugate();
  }
} Sensor;

// Config, written to EEPROM.
static constexpr struct Configuration {
  uint8_t channel{};
  bool    compass{true};

  V23D::Quaternion calibration;

  struct {
    bool    enabled{};
    uint8_t yaw{V2MIDI::CC::GeneralPurpose1 + 4};
    uint8_t pitch{V2MIDI::CC::GeneralPurpose1 + 5};
    uint8_t roll{V2MIDI::CC::GeneralPurpose1 + 6};
  } euler;

  struct {
    bool    enabled{};
    uint8_t x{V2MIDI::CC::GeneralPurpose1 + 7};
    uint8_t y{V2MIDI::CC::GeneralPurpose1 + 8};
    uint8_t z{V2MIDI::CC::GeneralPurpose1 + 9};
  } gyroscope;
} ConfigurationDefault;

static class Device : public V2Device {
public:
  Device() : V2Device() {
    metadata.vendor      = "Versio Duo";
    metadata.product     = "V2 axis";
    metadata.description = "Orientation Sensor";
    metadata.home        = "https://versioduo.com/#axis";
    system.download      = "https://versioduo.com/download";
    system.configure     = "https://versioduo.com/configure";
    usb.ports.standard   = 0;
    configuration        = {.version{2}, .size{sizeof(config)}, .data{&config}};
    help.device          = "Accelerometer, Gyroscope, Magentometer – MIDI Control Change messages with Quaternion or Euler data.";
    help.configuration   = "# Setup\n"
                           "The device can be mounted at any orientation or angle, the reference frame / axes can be aligned to the current "
                           "posion and stored in the device. This sequence identifies the Y axis (the axis of rotation between the button "
                           "clicks) and the Z axis (the gravity at the second step):\n"
                           "• The device, or the body it is mounted on, points upwards / to the sky. A double-click of the the button, the "
                           "LED turns green.\n"
                           "• The device points forward, the zero position. A single click, the LED turns orange. The axes of the sensor are "
                           "now aligned and the rotation reset to zero.\n"
                           "• Any single click will set the rotation to zero, but not change the reference frame.\n"
                           "• A long-press double-click will store the calibration in the device, the LED flashes purple.\n"
                           "A long-press triple-click erases the stored data, the LED flashes purple.\n"
                           "# Control Change Mapping\n"
                           "Holding the device button down, suppresses all Control Change messages. The CC numbers can be configured in "
                           "the Setting section. Pressing the blue button there, sends this single CC message; it can be used to map the CC "
                           "in the Audio Workstation.";
  }

  enum class CC {
    Quaternion        = V2MIDI::CC::GeneralPurpose1,
    Home              = V2MIDI::CC::Controller14,
    SaveConfiguration = V2MIDI::CC::Controller15,
    Light             = V2MIDI::CC::Controller89,
  };

  Configuration config{ConfigurationDefault};

  // Store the tip and home orientation in the EPROM.
  auto storeConfiguration() {
    config.calibration = Sensor.calibration();

    writeConfiguration();
    LED.splashHSV(0.3, V2Colour::Magenta, 0.8, 0.5);
  }

  auto silent(bool on = true) {
    _silent = on;
  }

private:
  bool _silent{};

  V2MIDI::CC::HighResolution<V2MIDI::CC::GeneralPurpose1, 4> _hires;
  uint32_t                                                   _usec{};
  struct {
    uint8_t w{};
    uint8_t x{};
    uint8_t y{};
    uint8_t z{};
  } _quaternion;

  struct {
    uint8_t yaw{};
    uint8_t roll{};
    uint8_t pitch{};
  } _euler;

  struct {
    uint8_t x{};
    uint8_t y{};
    uint8_t z{};
  } _gyroscope;

  V2MIDI::Packet _midi{};
  float          _lightMax{};

  auto handleReset() -> void override {
    _silent = false;
    _hires.reset();
    _lightMax   = 100.f / 127.f;
    _quaternion = {};
    _euler      = {};
    _gyroscope  = {};

    Wire.end();
    Wire.begin();
    Wire.setClock(1000000);
    Wire.setTimeout(1);

    LED.reset();
    updateBrightness();
    Sensor.setup(config.compass, config.calibration);
  }

  auto updateBrightness() -> void {
    float min{0.05};
    float max{0.75};
    float range{(max - min) * _lightMax};
    LED.setMaxBrightness(min + range);
    Sensor.updateLED();
  }

  auto allNotesOff() {
    // Send the current values when 'Stop' is pressed in audio workstation.
    _hires.send(this, config.channel, uint8_t(CC::Quaternion) + 0);
    _hires.send(this, config.channel, uint8_t(CC::Quaternion) + 1);
    _hires.send(this, config.channel, uint8_t(CC::Quaternion) + 2);
    _hires.send(this, config.channel, uint8_t(CC::Quaternion) + 3);

    if (config.euler.enabled) {
      send(_midi.setControlChange(config.channel, config.euler.yaw, _euler.yaw));
      send(_midi.setControlChange(config.channel, config.euler.pitch, _euler.pitch));
      send(_midi.setControlChange(config.channel, config.euler.roll, _euler.roll));
    }

    if (config.gyroscope.enabled) {
      send(_midi.setControlChange(config.channel, config.gyroscope.x, _gyroscope.x));
      send(_midi.setControlChange(config.channel, config.gyroscope.y, _gyroscope.y));
      send(_midi.setControlChange(config.channel, config.gyroscope.z, _gyroscope.z));
    }
  }

  auto handleLoop() -> void override {
    if ((uint32_t)(V2Base::getUsec() - _usec) < 20 * 1000)
      return;

    if (_silent)
      return;

    {
      auto q{Sensor.getRotation()};
      if (_hires.set(uint8_t(CC::Quaternion) + 0, (q.w + 1.f) * 8191.f))
        _hires.send(this, config.channel, uint8_t(CC::Quaternion) + 0);

      if (_hires.set(uint8_t(CC::Quaternion) + 1, (q.x + 1.f) * 8191.f))
        _hires.send(this, config.channel, uint8_t(CC::Quaternion) + 1);

      if (_hires.set(uint8_t(CC::Quaternion) + 2, (q.y + 1.f) * 8191.f))
        _hires.send(this, config.channel, uint8_t(CC::Quaternion) + 2);

      if (_hires.set(uint8_t(CC::Quaternion) + 3, (q.z + 1.f) * 8191.f))
        _hires.send(this, config.channel, uint8_t(CC::Quaternion) + 3);

      if (config.euler.enabled) {
        auto e{V23D::Euler::quaternion({q.w, q.y, q.x, -q.z})};
        if (auto y{uint8_t((e.yaw / std::numbers::pi_v<float> + 1.f) / 2.f * 127.f)}; _euler.yaw != y) {
          send(_midi.setControlChange(config.channel, config.euler.yaw, y));
          _euler.yaw = y;
        }

        if (auto p{uint8_t((e.pitch / std::numbers::pi_v<float> + 1.f) / 2.f * 127.f)}; _euler.pitch != p) {
          send(_midi.setControlChange(config.channel, config.euler.pitch, p));
          _euler.pitch = p;
        }

        if (auto r{uint8_t((e.roll / std::numbers::pi_v<float> + 1.f) / 2.f * 127.f)}; _euler.roll != r) {
          send(_midi.setControlChange(config.channel, config.euler.roll, r));
          _euler.roll = r;
        }
      }
    }

    if (config.gyroscope.enabled) {
      auto center{[](float v) -> uint8_t {
        return ceilf((std::clamp((v + 1.f) / 2.f, 0.f, 1.f)) * 127.f);
      }};

      auto g{Sensor.getAcceleration()};
      if (auto x{center(g.x)}; _gyroscope.x != x) {
        send(_midi.setControlChange(config.channel, config.gyroscope.x, x));
        _gyroscope.x = x;
      }

      if (auto y{center(g.y)}; _gyroscope.y != y) {
        send(_midi.setControlChange(config.channel, config.gyroscope.y, y));
        _gyroscope.y = y;
      }

      if (auto z{center(g.z)}; _gyroscope.z != z) {
        send(_midi.setControlChange(config.channel, config.gyroscope.z, z));
        _gyroscope.z = z;
      }
    }

    _usec = V2Base::getUsec();
  }

  auto handleSend(V2MIDI::Packet* midi) -> bool override {
    usb.midi.send(midi);
    return true;
  }

  auto handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) -> void override {
    switch (controller) {
      case uint8_t(CC::Home):
        Sensor.home();
        break;

      case uint8_t(CC::SaveConfiguration):
        storeConfiguration();
        break;

      case uint8_t(CC::Light):
        _lightMax = float(value) / 127.f;
        updateBrightness();
        break;

      case V2MIDI::CC::AllSoundOff:
      case V2MIDI::CC::AllNotesOff:
        allNotesOff();
        break;
    }
  }

  auto handleSystemExclusive(const uint8_t* buffer, uint32_t len) -> void override {
    if (len < 20)
      return;

    // 0x7d == SysEx prototype/research/private ID
    if (buffer[1] != 0x7d)
      return;

    // Handle only JSON messages.
    if (buffer[2] != '{' || buffer[len - 2] != '}')
      return;

    // Read incoming message.
    JsonDocument json;
    if (deserializeJson(json, buffer + 2, len - 1))
      return;

    JsonObject test{json["test"]};
    if (!test)
      return;

    if (test["controller"].isNull())
      return;

    send(_midi.setControlChange(config.channel, test["controller"], 64));
  }

  auto handleSystemReset() -> void override {
    reset();
  }

  void exportLinks(JsonArray json) override {
    JsonObject jsonLink     = json.add<JsonObject>();
    jsonLink["description"] = "Rotating Cube";

    char link[128] = "https://versioduo.com/axis3d?connect=";
    strlcat(link, usb.name ? usb.name : metadata.product, sizeof(link));
    jsonLink["target"] = link;
  }

  auto exportSystem(JsonObject json) -> void override {
    JsonObject jsonPower{json["sensor"].to<JsonObject>()};
    jsonPower["product"]  = Sensor.getProductID();
    jsonPower["revision"] = Sensor.getRevisionID();
    jsonPower["software"] = Sensor.getRAMVersion();
  }

  auto exportSettings(JsonArray json) -> void override {
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
      JsonObject setting{json.add<JsonObject>()};
      setting["type"]  = "toggle";
      setting["title"] = "Sensor";
      setting["label"] = "Mode";
      setting["text"]  = "Compass";
      setting["path"]  = "compass";
    }

    {
      JsonObject setting{json.add<JsonObject>()};
      setting["type"]  = "toggle";
      setting["ruler"] = true;
      setting["text"]  = "Euler";
      setting["label"] = "Messages";
      setting["path"]  = "euler/enabled";
    }
    {
      JsonObject setting{json.add<JsonObject>()};
      setting["type"]    = "controller";
      setting["label"]   = "Yaw";
      setting["test"]    = true;
      setting["path"]    = "euler/yaw";
      setting["default"] = ConfigurationDefault.euler.yaw;
    }
    {
      JsonObject setting{json.add<JsonObject>()};
      setting["type"]    = "controller";
      setting["label"]   = "Pitch";
      setting["test"]    = true;
      setting["path"]    = "euler/pitch";
      setting["default"] = ConfigurationDefault.euler.pitch;
    }
    {
      JsonObject setting{json.add<JsonObject>()};
      setting["type"]    = "controller";
      setting["label"]   = "Roll";
      setting["test"]    = true;
      setting["path"]    = "euler/roll";
      setting["default"] = ConfigurationDefault.euler.roll;
    }

    {
      JsonObject setting{json.add<JsonObject>()};
      setting["type"]  = "toggle";
      setting["ruler"] = true;
      setting["text"]  = "Gyroscope";
      setting["label"] = "Messages";
      setting["test"]  = true;
      setting["path"]  = "gyroscope/enabled";
    }
    {
      JsonObject setting{json.add<JsonObject>()};
      setting["type"]    = "controller";
      setting["label"]   = "X";
      setting["test"]    = true;
      setting["path"]    = "gyroscope/x";
      setting["default"] = ConfigurationDefault.gyroscope.x;
    }
    {
      JsonObject setting{json.add<JsonObject>()};
      setting["type"]    = "controller";
      setting["label"]   = "Y";
      setting["test"]    = true;
      setting["path"]    = "gyroscope/y";
      setting["default"] = ConfigurationDefault.gyroscope.y;
    }
    {
      JsonObject setting{json.add<JsonObject>()};
      setting["type"]    = "controller";
      setting["label"]   = "Z";
      setting["test"]    = true;
      setting["path"]    = "gyroscope/z";
      setting["default"] = ConfigurationDefault.gyroscope.z;
    }
  }

  auto importConfiguration(JsonObject json) -> void override {
    JsonObject jsonMidi{json["midi"]};
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

    JsonObject jsonCalibration{json["calibration"]};
    if (jsonCalibration) {
      config.calibration.w = jsonCalibration["w"];
      config.calibration.x = jsonCalibration["x"];
      config.calibration.y = jsonCalibration["y"];
      config.calibration.z = jsonCalibration["z"];
    }

    JsonObject jsonEuler{json["euler"]};
    if (jsonEuler) {
      if (!jsonEuler["enabled"].isNull())
        config.euler.enabled = jsonEuler["enabled"];

      config.euler.yaw   = jsonEuler["yaw"];
      config.euler.pitch = jsonEuler["pitch"];
      config.euler.roll  = jsonEuler["roll"];
    }

    JsonObject jsonGyroscope{json["gyroscope"]};
    if (jsonGyroscope) {
      if (!jsonGyroscope["enabled"].isNull())
        config.gyroscope.enabled = jsonGyroscope["enabled"];

      config.gyroscope.x = jsonGyroscope["x"];
      config.gyroscope.y = jsonGyroscope["y"];
      config.gyroscope.z = jsonGyroscope["z"];
    }

    reset();
  }

  auto exportConfiguration(JsonObject json) -> void override {
    {
      json["#midi"] = "The MIDI settings";
      auto m{json["midi"].to<JsonObject>()};
      m["#channel"] = "The channel to send notes and control values to";
      m["channel"]  = config.channel + 1;
    }

    json["#compass"] = "Use the compass for absolute coordinates";
    json["compass"]  = config.compass;

    {
      auto c{json["calibration"].to<JsonObject>()};
      c["#"] = "The rotation of the sensor to the device's zero position";
      c["w"] = serialized(String(config.calibration.w, 4));
      c["x"] = serialized(String(config.calibration.x, 4));
      c["y"] = serialized(String(config.calibration.y, 4));
      c["z"] = serialized(String(config.calibration.z, 4));
    }
    {
      auto e{json["euler"].to<JsonObject>()};
      e["#"]       = "The controller numbers for the Euler coordinates";
      e["enabled"] = config.euler.enabled;
      e["yaw"]     = config.euler.yaw;
      e["pitch"]   = config.euler.pitch;
      e["roll"]    = config.euler.roll;
    }
    {
      auto g{json["gyroscope"].to<JsonObject>()};
      g["#"]       = "The controller numbers for the gyroscope";
      g["enabled"] = config.gyroscope.enabled;
      g["x"]       = config.gyroscope.x;
      g["y"]       = config.gyroscope.y;
      g["z"]       = config.gyroscope.z;
    }
  }

  auto exportInput(JsonObject json) -> void override {
    auto jsonControllers{json["controllers"].to<JsonArray>()};
    {
      auto c{jsonControllers.add<JsonObject>()};
      c["name"]   = "Home";
      c["type"]   = "momentary";
      c["number"] = uint8_t(CC::Home);
    }
    {
      auto c{jsonControllers.add<JsonObject>()};
      c["name"]   = "Save";
      c["type"]   = "momentary";
      c["number"] = uint8_t(CC::SaveConfiguration);
    }
    {
      auto c{jsonControllers.add<JsonObject>()};
      c["name"]   = "Brightness";
      c["number"] = uint8_t(CC::Light);
      c["value"]  = uint8_t(_lightMax) * 127.f;
    }
  }

  auto exportOutput(JsonObject json) -> void override {
    json["channel"] = config.channel;

    auto jsonControllers{json["controllers"].to<JsonArray>()};
    {
      auto c{jsonControllers.add<JsonObject>()};
      c["name"]      = "Quaternion W";
      c["number"]    = uint8_t(CC::Quaternion) + 0;
      c["value"]     = _hires.getMSB(uint8_t(CC::Quaternion) + 0);
      c["valueFine"] = _hires.getLSB(uint8_t(CC::Quaternion) + 0);
    }
    {
      auto c{jsonControllers.add<JsonObject>()};
      c["name"]      = "Quaternion X";
      c["number"]    = uint8_t(CC::Quaternion) + 1;
      c["value"]     = _hires.getMSB(uint8_t(CC::Quaternion) + 1);
      c["valueFine"] = _hires.getLSB(uint8_t(CC::Quaternion) + 1);
    }
    {
      auto c{jsonControllers.add<JsonObject>()};
      c["name"]      = "Quaternion Y";
      c["number"]    = uint8_t(CC::Quaternion) + 2;
      c["value"]     = _hires.getMSB(uint8_t(CC::Quaternion) + 2);
      c["valueFine"] = _hires.getLSB(uint8_t(CC::Quaternion) + 2);
    }
    {
      auto c{jsonControllers.add<JsonObject>()};
      c["name"]      = "Quaternion Z";
      c["number"]    = uint8_t(CC::Quaternion) + 3;
      c["value"]     = _hires.getMSB(uint8_t(CC::Quaternion) + 3);
      c["valueFine"] = _hires.getLSB(uint8_t(CC::Quaternion) + 3);
    }

    if (config.euler.enabled) {
      {
        auto c{jsonControllers.add<JsonObject>()};
        c["name"]   = "Euler Yaw";
        c["number"] = config.euler.yaw;
        c["value"]  = _euler.yaw;
      }
      {
        auto c{jsonControllers.add<JsonObject>()};
        c["name"]   = "Euler Pitch";
        c["number"] = config.euler.pitch;
        c["value"]  = _euler.pitch;
      }
      {
        auto c{jsonControllers.add<JsonObject>()};
        c["name"]   = "Euler Roll";
        c["number"] = config.euler.roll;
        c["value"]  = _euler.roll;
      }
    }

    if (config.gyroscope.enabled) {
      {
        auto c{jsonControllers.add<JsonObject>()};
        c["name"]   = "Gyroscope X";
        c["number"] = config.gyroscope.x;
        c["value"]  = _gyroscope.x;
      }
      {
        auto c{jsonControllers.add<JsonObject>()};
        c["name"]   = "Gyroscope Y";
        c["number"] = config.gyroscope.y;
        c["value"]  = _gyroscope.y;
      }
      {
        auto c{jsonControllers.add<JsonObject>()};
        c["name"]   = "Gyroscope Z";
        c["number"] = config.gyroscope.z;
        c["value"]  = _gyroscope.z;
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

  auto handleUp() -> void {
    Device.silent(false);
  }

  auto handleHold(uint8_t count) -> void override {
    switch (count) {
      case 0:
        Device.silent();
        break;

      case 1:
        Device.storeConfiguration();
        break;

      case 2:
        Device.config.calibration = {};
        Device.reset();
        Device.storeConfiguration();
        break;
    }
  }

  auto handleClick(uint8_t count) -> void override {
    switch (count) {
      case 0:
        Sensor.home();
        break;

      case 1:
        Sensor.tip();
        break;
    }
  }
} Button;

auto setup() -> void {
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

auto loop() -> void {
  LED.loop();
  MIDI.loop();
  Sensor.loop();
  V2Buttons::loop();
  Device.loop();

  if (Device.idle())
    Device.sleep();
}
