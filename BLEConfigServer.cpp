#include "BLEConfigServer.h"

BLEConfigServer::BLEConfigServer() {}

void BLEConfigServer::begin(ISGConfigListener *listener) {
  _listener = listener;
  NimBLEDevice::init(SG_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Max power (adjust if needed)
  // (Optional) Connection parameter tuning not applied; NimBLE-Arduino stable API lacks direct setConnectionParams here.

  _server = NimBLEDevice::createServer();
  // Register server callbacks to ensure advertising restarts after disconnects
  if (!_serverCallbacks) {
    _serverCallbacks = new ServerCallbacks(this);
  }
  _server->setCallbacks(_serverCallbacks);
  _service = _server->createService(SG_SERVICE_UUID);

  // Speed characteristic (float). We'll exchange ASCII string representation for simplicity.
  _speedChar = _service->createCharacteristic(
      SG_SPEED_CHAR_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  _speedChar->setValue(String(_speedMultiplier, 3).c_str());
  _speedChar->setCallbacks(new SpeedCallbacks(this));

  // Pattern characteristic (int)
  _patternChar = _service->createCharacteristic(
      SG_PATTERN_CHAR_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  _patternChar->setValue(String(_currentPattern).c_str());
  _patternChar->setCallbacks(new PatternCallbacks(this));

  // Optional status notify characteristic
  _statusChar = _service->createCharacteristic(
      SG_STATUS_CHAR_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  _statusChar->setValue("Boot");

  // Mode characteristic (0 manual, 1 automatic)
  _modeChar = _service->createCharacteristic(
    SG_MODE_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  _modeChar->setValue(String(_autoMode ? 1 : 0).c_str());
  _modeChar->setCallbacks(new ModeCallbacks(this));

  // Run characteristic (0 stopped, 1 running)
  _runChar = _service->createCharacteristic(
    SG_RUN_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  _runChar->setValue(String(_runState ? 1 : 0).c_str());
  _runChar->setCallbacks(new RunCallbacks(this));

  // Telemetry characteristic (notify-only)
  _telemetryChar = _service->createCharacteristic(
    SG_TELEMETRY_CHAR_UUID,
    NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ); // allow a last snapshot read
  _telemetryChar->setValue("TL0");

  // Command characteristic (write-only from client, readable for last command echo/result)
  _commandChar = _service->createCharacteristic(
    SG_COMMAND_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
  _commandChar->setValue("READY");
  _commandChar->setCallbacks(new CommandCallbacks(this));

  _service->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  // --- Advertising configuration ---
  // ISSUE (Windows not showing name until connect):
  // The original code tried to place Flags + 128-bit Service UUID + Complete Local Name
  // all in the primary advertising packet. Size math (BLE legacy ADV max = 31 bytes):
  //   Flags AD structure:            3 bytes (len=2,type=0x01,data=1)
  //   128-bit Service UUID structure: 18 bytes (len=17,type=0x07,data=16)
  //   Complete Local Name ("Sand Garden") structure: 13 bytes (len=12,type=0x09,data=11)
  //   TOTAL = 34 bytes > 31 -> stack silently drops/moves one field (usually the name
  //   goes to the scan response). Windows' default UI often does NOT perform a scan
  //   response request before showing devices, so the name appeared only after connect.
  // FIX: Put only Flags + Complete Local Name in the primary ADV, move the 128-bit
  // Service UUID to the scan response. This guarantees name visibility on Windows.
  // If you later need service UUID in the primary packet, shorten the name or use a
  // 16-bit SIG-adopted UUID (not possible for custom service) or switch to extended
  // advertising (not broadly supported in all stacks yet).

  NimBLEAdvertisementData advData;  // primary advertising payload (keep minimal for name)
  NimBLEAdvertisementData scanData; // secondary scan response (roomy for UUIDs, etc.)

  // Flags (LE General Discoverable, BR/EDR not supported)
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);

  // Put the COMPLETE LOCAL NAME in the primary packet so Windows shows it immediately.
  advData.setName(SG_DEVICE_NAME);

  // Move the (large) 128-bit custom service UUID to the scan response to stay under 31 bytes.
  scanData.addServiceUUID(_service->getUUID());

  // Optional: appearance (generic = 0)
  adv->setAppearance(0x0000);

  // Apply assembled payloads.
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);

  // (Optional future enhancement) You can add manufacturer data or additional
  // characteristics counts to scanData later, but keep primary adv lean.

  // Start advertising (auto-restart on disconnect is enabled by default in NimBLE-Arduino)
  adv->start();
}

void BLEConfigServer::loop() {
  _watchdog();
}

void BLEConfigServer::setSpeedMultiplier(float v) {
  if (v <= 0) v = 0.01f; // avoid zero or negative
  if (fabsf(v - _speedMultiplier) < 0.0001f) return; // no change
  _speedMultiplier = v;
  if (_speedChar) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f", _speedMultiplier);
    _speedChar->setValue(buf);
    _speedChar->notify();
  }
  if (_listener) _listener->onSpeedMultiplierChanged(_speedMultiplier);
}

void BLEConfigServer::setCurrentPattern(int p) {
  if (p < 1) p = 1; // caller responsible for clamping to available patterns
  if (p == _currentPattern) return;
  _currentPattern = p;
  if (_patternChar) {
    _patternChar->setValue(String(_currentPattern).c_str());
    _patternChar->notify();
  }
  if (_listener) _listener->onCurrentPatternChanged(_currentPattern);
}

void BLEConfigServer::setAutoMode(bool m) {
  if (_autoMode == m) return;
  _autoMode = m;
  if (_modeChar) {
    _modeChar->setValue(String(_autoMode ? 1 : 0).c_str());
    _modeChar->notify();
  }
  if (_listener) _listener->onAutoModeChanged(_autoMode);
}

void BLEConfigServer::setRunState(bool r) {
  if (_runState == r) return;
  _runState = r;
  if (_runChar) {
    _runChar->setValue(String(_runState ? 1 : 0).c_str());
    _runChar->notify();
  }
  if (_listener) _listener->onRunStateChanged(_runState);
}

void BLEConfigServer::notifyStatus(const String &msg) {
  if (_statusChar) {
    _statusChar->setValue(std::string(msg.c_str()));
    _statusChar->notify();
  }
}

void BLEConfigServer::notifyTelemetry(const String &msg) {
  if (_telemetryChar) {
    _telemetryChar->setValue(std::string(msg.c_str()));
    _telemetryChar->notify();
  }
}

void BLEConfigServer::restartAdvertising(const char *reasonTag) {
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  if (!adv) return;
  if (!NimBLEDevice::getServer() || NimBLEDevice::getServer()->getConnectedCount() > 0) return; // still connected
  // Throttle attempts to at most once every 2 seconds
  uint32_t now = millis();
  if (now - _lastAdvAttemptMs < 2000) return;
  _lastAdvAttemptMs = now;
  if (reasonTag && _statusChar) {
    notifyStatus(String("[BLE] ADV_RESTART reason=") + reasonTag);
  }
  adv->start();
}

void BLEConfigServer::disconnectAll(const char *reasonTag) {
  NimBLEServer *srv = NimBLEDevice::getServer();
  if (!srv) return;
  if (reasonTag && _statusChar) notifyStatus(String("[BLE] BLEDROP reason=") + reasonTag);
  // Copy handles because disconnect alters list
  auto handles = _connHandles;
  for (auto h : handles) {
    srv->disconnect(h);
  }
  _connHandles.clear();
  restartAdvertising("drop");
}

void BLEConfigServer::_watchdog() {
  // If no active connections and not advertising, attempt restart.
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  NimBLEServer *srv = NimBLEDevice::getServer();
  if (!adv || !srv) return;
  bool anyConn = srv->getConnectedCount() > 0;
  if (!anyConn && !adv->isAdvertising()) {
    restartAdvertising("wd");
  }
}

void BLEConfigServer::_applySpeedWrite(const std::string &valRaw) {
  // Accept either ASCII float or raw 4-byte float
  float newVal = _speedMultiplier;
  if (valRaw.size() == 4) {
    // interpret bytes as float (little endian from client assumed)
    float temp;
    memcpy(&temp, valRaw.data(), 4);
    newVal = temp;
  } else {
    // treat as string
    newVal = atof(valRaw.c_str());
  }
  setSpeedMultiplier(newVal);
}

void BLEConfigServer::_applyPatternWrite(const std::string &valRaw) {
  int newVal = _currentPattern;
  if (valRaw.size() == 4) {
    int temp;
    memcpy(&temp, valRaw.data(), 4);
    newVal = temp;
  } else {
    newVal = atoi(valRaw.c_str());
  }
  setCurrentPattern(newVal);
}

void BLEConfigServer::_applyModeWrite(const std::string &valRaw) {
  int v = atoi(valRaw.c_str());
  setAutoMode(v != 0);
}

void BLEConfigServer::_applyRunWrite(const std::string &valRaw) {
  int v = atoi(valRaw.c_str());
  setRunState(v != 0);
}

void BLEConfigServer::_applyCommandWrite(const std::string &valRaw) {
  // Copy & sanitize: trim whitespace, uppercase token (first word)
  std::string raw = valRaw;
  // Trim leading/trailing whitespace
  auto ltrim=[&](std::string &s){ while(!s.empty() && isspace((unsigned char)s.front())) s.erase(s.begin());};
  auto rtrim=[&](std::string &s){ while(!s.empty() && isspace((unsigned char)s.back())) s.pop_back();};
  ltrim(raw); rtrim(raw);
  std::string token = raw;
  // For future args support: split at first space
  size_t sp = raw.find(' ');
  if (sp != std::string::npos) token = raw.substr(0, sp);
  for(char &c : token) c = toupper((unsigned char)c);
  if (_commandChar) {
    // Echo back sanitized token (could also include OK/ERR later)
    _commandChar->setValue(token);
  }
  if (_listener) {
    _listener->onCommandReceived(String(token.c_str()), valRaw);
  }
  // Built-in commands
  if (token == "BLEADV") {
    restartAdvertising("cmd");
  } else if (token == "BLEDROP") {
    disconnectAll("cmd");
  }
}

void BLEConfigServer::SpeedCallbacks::onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) {
  (void)info; // unused
  _parent->_applySpeedWrite(c->getValue());
}

void BLEConfigServer::PatternCallbacks::onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) {
  (void)info;
  _parent->_applyPatternWrite(c->getValue());
}

void BLEConfigServer::ModeCallbacks::onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) {
  (void)info;
  _parent->_applyModeWrite(c->getValue());
}

void BLEConfigServer::RunCallbacks::onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) {
  (void)info;
  _parent->_applyRunWrite(c->getValue());
}

void BLEConfigServer::CommandCallbacks::onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) {
  (void)info;
  _parent->_applyCommandWrite(c->getValue());
}
