#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

// BLE Service & Characteristic UUIDs (randomly generated v4 UUIDs)
// Single service exposes configuration for Sand Garden
#define SG_SERVICE_UUID            "9b6c7e10-3b2c-4d8c-9d7c-5e2a6d1f8b01"
#define SG_SPEED_CHAR_UUID         "9b6c7e11-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // float speed multiplier
#define SG_PATTERN_CHAR_UUID       "9b6c7e12-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // int current pattern index
#define SG_STATUS_CHAR_UUID        "9b6c7e13-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // notify-only status (optional)
#define SG_MODE_CHAR_UUID          "9b6c7e14-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // 0 manual, 1 automatic
#define SG_RUN_CHAR_UUID           "9b6c7e15-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // 0 stopped, 1 running
#define SG_TELEMETRY_CHAR_UUID      "9b6c7e16-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // streaming telemetry (notify)

// Name of the BLE peripheral
#define SG_DEVICE_NAME "Sand Garden"

// Callback interface for host sketch to observe updates
class ISGConfigListener {
public:
  virtual void onSpeedMultiplierChanged(float newValue) = 0;
  virtual void onCurrentPatternChanged(int newPattern) = 0;
  virtual void onAutoModeChanged(bool newAutoMode) = 0;
  virtual void onRunStateChanged(bool newRunState) = 0;
};

class BLEConfigServer {
public:
  BLEConfigServer();
  void begin(ISGConfigListener *listener = nullptr);
  void loop();

  // Accessors
  float speedMultiplier() const { return _speedMultiplier; }
  int currentPattern() const { return _currentPattern; }

  // Mutators (will also update BLE characteristics + notify clients)
  void setSpeedMultiplier(float v);
  void setCurrentPattern(int p);
  void setAutoMode(bool m);
  void setRunState(bool r);
  void notifyStatus(const String &msg);
  void notifyTelemetry(const String &msg);

private:
  class SpeedCallbacks : public NimBLECharacteristicCallbacks {
  public:
    SpeedCallbacks(BLEConfigServer *parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override; // signature per NimBLE-Arduino 2.x
  private:
    BLEConfigServer *_parent;
  };
  class PatternCallbacks : public NimBLECharacteristicCallbacks {
  public:
    PatternCallbacks(BLEConfigServer *parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override;
  private:
    BLEConfigServer *_parent;
  };
  class ModeCallbacks : public NimBLECharacteristicCallbacks {
  public:
    ModeCallbacks(BLEConfigServer *parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override;
  private:
    BLEConfigServer *_parent;
  };
  class RunCallbacks : public NimBLECharacteristicCallbacks {
  public:
    RunCallbacks(BLEConfigServer *parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override;
  private:
    BLEConfigServer *_parent;
  };

  void _applySpeedWrite(const std::string &valRaw);
  void _applyPatternWrite(const std::string &valRaw);
  void _applyModeWrite(const std::string &valRaw);
  void _applyRunWrite(const std::string &valRaw);

  NimBLEServer *_server = nullptr;
  NimBLEService *_service = nullptr;
  NimBLECharacteristic *_speedChar = nullptr;
  NimBLECharacteristic *_patternChar = nullptr;
  NimBLECharacteristic *_statusChar = nullptr; // notify/read
  NimBLECharacteristic *_modeChar = nullptr;   // read/write/notify
  NimBLECharacteristic *_runChar = nullptr;    // read/write/notify
  NimBLECharacteristic *_telemetryChar = nullptr; // notify-only telemetry

  ISGConfigListener *_listener = nullptr;

  float _speedMultiplier = 1.0f; // default neutral multiplier
  int _currentPattern = 1;        // maps to sketch patterns (1-indexed like existing code)
  bool _autoMode = true;
  bool _runState = false;
};
