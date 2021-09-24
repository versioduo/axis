// © Kay Sievers <kay@vrfy.org>, 2020-2021
// SPDX-License-Identifier: Apache-2.0

#include <V2BNO055.h>
#include <V2Color.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <Wire.h>

V2DEVICE_METADATA("com.versioduo.axis", 13, "versioduo:samd:axis");

static V2LED::WS2812 LED(1, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
static V2Link::Port Plug(&SerialPlug);
static V2Link::Port Socket(&SerialSocket);
static V2BNO055 Sensor(&Wire, PIN_SENSOR_RESET, PIN_SENSOR_INTERRUPT);

static class Device : public V2Device {
public:
  Device() : V2Device() {
    metadata.vendor      = "Versio Duo";
    metadata.product     = "axis";
    metadata.description = "Orientation Sensor";
    metadata.home        = "https://versioduo.com/#axis";

    system.download  = "https://versioduo.com/download";
    system.configure = "https://versioduo.com/configure";

    configuration = {.magic{0x9e020000 | usb.pid}, .size{sizeof(config)}, .data{&config}};
  }

  // Config, written to EEPROM
  struct {
    uint8_t channel;
    struct {
      uint8_t heading;
      uint8_t roll;
      uint8_t pitch;
    } orientation;
  } config{.channel{},
           .orientation{.heading = V2MIDI::CC::GeneralPurpose1 + 0,
                        .roll    = V2MIDI::CC::GeneralPurpose1 + 1,
                        .pitch   = V2MIDI::CC::GeneralPurpose1 + 2}};

private:
  long _usec = 0;
  struct {
    uint8_t heading;
    uint8_t roll;
    uint8_t pitch;
  } _orientation{};
  V2MIDI::Packet _midi{};

  void handleReset() override {
    digitalWrite(PIN_LED_ONBOARD, LOW);
    LED.reset();
    _orientation = {};
  }

  void allNotesOff() {
    // Send the current values when 'Stop' is pressed in audio workstation.
    send(_midi.setControlChange(config.channel, config.orientation.heading, _orientation.heading));
    send(_midi.setControlChange(config.channel, config.orientation.roll, _orientation.roll));
    send(_midi.setControlChange(config.channel, config.orientation.pitch, _orientation.pitch));
  }

  void handleLoop() override {
    if ((unsigned long)(micros() - _usec) < 50 * 1000)
      return;

    float h, r, p;
    Sensor.readEuler(h, r, p);

    // Map the degree ranges to MIDI Control values.
    uint8_t heading = (h / 360) * 127;
    uint8_t roll    = ((r + 90) / 180) * 127;
    uint8_t pitch   = ((p + 180) / 360) * 127;

    if (heading != _orientation.heading) {
      LED.setHSV(0, V2Color::Red, 1, (float)heading / 127);
      send(_midi.setControlChange(config.channel, config.orientation.heading, heading));
      _orientation.heading = heading;
    }

    if (roll != _orientation.roll) {
      LED.setHSV(0, V2Color::Green, 1, (float)roll / 127);
      send(_midi.setControlChange(config.channel, config.orientation.roll, roll));
      _orientation.roll = roll;
    }

    if (pitch != _orientation.pitch) {
      LED.setHSV(0, V2Color::Blue, 1, (float)pitch / 127);
      send(_midi.setControlChange(config.channel, config.orientation.pitch, pitch));
      _orientation.pitch = pitch;
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
      case V2MIDI::CC::AllSoundOff:
      case V2MIDI::CC::AllNotesOff:
        allNotesOff();
        break;
    }
  }

  void handleSystemReset() override {
    reset();
  }

  void exportSettings(JsonArray json) override {
    JsonObject json_midi = json.createNestedObject();
    json_midi["type"]    = "midi";
    json_midi["channel"] = "midi.channel";

    // The object in the configuration record.
    JsonObject json_configuration = json_midi.createNestedObject("configuration");
    json_configuration["path"]    = "midi";
    json_configuration["field"]   = "channel";
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
  }

  void exportConfiguration(JsonObject json) override {
    json["#midi"]         = "The MIDI settings";
    JsonObject json_midi  = json.createNestedObject("midi");
    json_midi["#channel"] = "The channel to send notes and control values to";
    json_midi["channel"]  = config.channel + 1;
  }

  void exportOutput(JsonObject json) override {
    json["channel"] = config.channel;

    JsonArray json_controllers          = json.createNestedArray("controllers");
    JsonObject json_orientation_heading = json_controllers.createNestedObject();
    json_orientation_heading["name"]    = "Heading";
    json_orientation_heading["number"]  = config.orientation.heading;
    JsonObject json_orientation_roll    = json_controllers.createNestedObject();
    json_orientation_roll["name"]       = "Roll";
    json_orientation_roll["number"]     = config.orientation.roll;
    JsonObject json_orientation_pitch   = json_controllers.createNestedObject();
    json_orientation_pitch["name"]      = "Pitch";
    json_orientation_pitch["number"]    = config.orientation.pitch;
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

void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(1);

  LED.begin();
  LED.setMaxBrightness(0.5);

  Plug.begin();
  Socket.begin();
  Device.begin();

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. This needs to be after begin().
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 2);

  // Uses delay(), needs to be after Device.begin();
  Sensor.begin();
  Device.reset();
}

void loop() {
  LED.loop();
  MIDI.loop();
  Link.loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
