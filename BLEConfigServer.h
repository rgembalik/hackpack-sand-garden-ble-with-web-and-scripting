// ---------------------------------------------------------------------------------------------------------------------
// BLEConfigServer.h - BLE service abstraction for Sand Garden
// Adds a generic COMMAND characteristic to trigger actions (e.g. LED self-test) at runtime.
// ---------------------------------------------------------------------------------------------------------------------
#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <vector>
#include <algorithm>

// BLE Service & Characteristic UUIDs (randomly generated v4 UUIDs)
// Single service exposes configuration for Sand Garden
#define SG_SERVICE_UUID             "9b6c7e10-3b2c-4d8c-9d7c-5e2a6d1f8b01"
#define SG_SPEED_CHAR_UUID          "9b6c7e11-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // float speed multiplier
#define SG_PATTERN_CHAR_UUID        "9b6c7e12-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // int current pattern index
#define SG_STATUS_CHAR_UUID         "9b6c7e13-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // status / log lines (read + notify)
#define SG_MODE_CHAR_UUID           "9b6c7e14-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // 0 manual, 1 automatic
#define SG_RUN_CHAR_UUID            "9b6c7e15-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // 0 stopped, 1 running
#define SG_TELEMETRY_CHAR_UUID      "9b6c7e16-3b2c-4d8c-9d7c-5e2a6d1f8b01"  // telemetry snapshots / streaming
// Generic command channel (ASCII). Client writes commands like "SELFTEST"; device responds via status/telemetry.
#define SG_COMMAND_CHAR_UUID        "9b6c7e17-3b2c-4d8c-9d7c-5e2a6d1f8b01"

// Name of the BLE peripheral
#define SG_DEVICE_NAME "Sand Garden"

// Callback interface for host sketch to observe updates
class ISGConfigListener {
public:
  virtual void onSpeedMultiplierChanged(float newValue) = 0;
  virtual void onCurrentPatternChanged(int newPattern) = 0;
  virtual void onAutoModeChanged(bool newAutoMode) = 0;
  virtual void onRunStateChanged(bool newRunState) = 0;
  // Optional override for generic commands (uppercased token). Default no-op keeps backward compatibility.
  // 'cmd' is the uppercase trimmed command token; 'raw' holds the original payload (could include arguments later).
  virtual void onCommandReceived(const String &cmd, const std::string &raw) { (void)cmd; (void)raw; }
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
  // Manual controls (invoked via command characteristic tokens or future API)
  void restartAdvertising(const char *reasonTag = nullptr);
  void disconnectAll(const char *reasonTag = nullptr);

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
  class CommandCallbacks : public NimBLECharacteristicCallbacks {
  public:
    CommandCallbacks(BLEConfigServer *parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override;
  private:
    BLEConfigServer *_parent;
  };

  void _applySpeedWrite(const std::string &valRaw);
  void _applyPatternWrite(const std::string &valRaw);
  void _applyModeWrite(const std::string &valRaw);
  void _applyRunWrite(const std::string &valRaw);
  void _applyCommandWrite(const std::string &valRaw);

  NimBLEServer *_server = nullptr;
  NimBLEService *_service = nullptr;
  NimBLECharacteristic *_speedChar = nullptr;
  NimBLECharacteristic *_patternChar = nullptr;
  NimBLECharacteristic *_statusChar = nullptr;    // notify/read
  NimBLECharacteristic *_modeChar = nullptr;      // read/write/notify
  NimBLECharacteristic *_runChar = nullptr;       // read/write/notify
  NimBLECharacteristic *_telemetryChar = nullptr; // notify/read
  NimBLECharacteristic *_commandChar = nullptr;   // write/read

  ISGConfigListener *_listener = nullptr;

  float _speedMultiplier = 1.0f; // default neutral multiplier
  int _currentPattern = 1;       // maps to sketch patterns (1-indexed like existing code)
  bool _autoMode = true;
  bool _runState = false;

  // Server connection lifecycle callbacks (restart advertising after disconnect)
  class ServerCallbacks : public NimBLEServerCallbacks {
  public:
    ServerCallbacks(BLEConfigServer *parent) : _parent(parent) {}
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
      (void)server; (void)connInfo;
      // Optional: could lower TX power or adjust parameters here.
      if (_parent && _parent->_statusChar) {
        _parent->notifyStatus(String("[BLE] CONNECT conn=") + String(connInfo.getConnHandle()));
      }
      if (_parent) {
        uint16_t h = connInfo.getConnHandle();
        bool found=false; for(auto v : _parent->_connHandles){ if(v==h){found=true;break;} }
        if(!found) _parent->_connHandles.push_back(h);
      }
    }
    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
      (void)server;
      if (_parent && _parent->_statusChar) {
        _parent->notifyStatus(String("[BLE] DISCONNECT reason=") + String(reason));
      }
      if (_parent) {
        // Remove handle from list
        uint16_t h = connInfo.getConnHandle();
        auto &vec = _parent->_connHandles;
        vec.erase(std::remove(vec.begin(), vec.end(), h), vec.end());
        _parent->restartAdvertising("disc");
      }
    }
    void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override {
      (void)connInfo;
      if (_parent && _parent->_statusChar) {
        _parent->notifyStatus(String("[BLE] MTU=") + String(MTU));
      }
    }
  private:
    BLEConfigServer *_parent;
  };
  ServerCallbacks *_serverCallbacks = nullptr; // keep pointer to manage lifetime

  // Internal helpers
  // (moved) _applyCommandWrite declared only once here (removed earlier duplicate)
  void _watchdog();

  std::vector<uint16_t> _connHandles; // active connections
  uint32_t _lastAdvAttemptMs = 0;     // throttle advertising restart attempts
};
