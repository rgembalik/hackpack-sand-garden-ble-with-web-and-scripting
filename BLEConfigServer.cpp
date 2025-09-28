#include "BLEConfigServer.h"
#include "PatternScript.h"

static const uint32_t SCRIPT_TRANSFER_TIMEOUT_MS = 5000;

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

  // Script chunk characteristic (binary-safe write sink for DSL source)
  _scriptChar = _service->createCharacteristic(
    SG_SCRIPT_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  _scriptChar->setValue("");
  _scriptChar->setCallbacks(new ScriptCallbacks(this));

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
  if (_scriptActive && _scriptLastChunkMs > 0) {
    uint32_t now = millis();
    if ((now - _scriptLastChunkMs) > SCRIPT_TRANSFER_TIMEOUT_MS) {
      String msg = String("[SCRIPT] ERR timeout after ") + String(static_cast<unsigned long>(now - _scriptLastChunkMs)) + "ms";
      notifyStatus(msg);
      if (_listener) _listener->onPatternScriptStatus(msg);
      _resetScriptTransfer("timeout", false);
    }
  }
  if (_scriptActive && _scriptProgressDirty) {
    uint32_t now = millis();
    if (_scriptLastProgressNotifyMs == 0 || (now - _scriptLastProgressNotifyMs) >= 750) {
      _scriptLastProgressNotifyMs = now;
      _scriptProgressDirty = false;
      if (_scriptExpectedLen > 0) {
        uint32_t percent = static_cast<uint32_t>((_scriptReceivedLen * 100UL) / _scriptExpectedLen);
        String msg = String("[SCRIPT] PROGRESS recv=") +
                     String(static_cast<unsigned long>(_scriptReceivedLen)) +
                     "/" + String(static_cast<unsigned long>(_scriptExpectedLen)) +
                     " (" + String(percent) + "%)";
        notifyStatus(msg);
        if (_listener) _listener->onPatternScriptStatus(msg);
      }
    }
  }
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
  std::string payload;
  // For future args support: split at first space
  size_t sp = raw.find(' ');
  if (sp != std::string::npos) {
    token = raw.substr(0, sp);
    payload = raw.substr(sp + 1);
    auto trimPayload=[&](std::string &s){ while(!s.empty() && isspace((unsigned char)s.front())) s.erase(s.begin()); while(!s.empty() && isspace((unsigned char)s.back())) s.pop_back(); };
    trimPayload(payload);
  }
  for(char &c : token) c = toupper((unsigned char)c);
  if (_commandChar) {
    // Echo back sanitized token (could also include OK/ERR later)
    _commandChar->setValue(token);
  }
  notifyStatus(String("[CMD] RX ") + token.c_str());
  if (_listener) {
    _listener->onCommandReceived(String(token.c_str()), valRaw);
  }
  // Built-in commands
  if (token == "BLEADV") {
    restartAdvertising("cmd");
  } else if (token == "BLEDROP") {
    disconnectAll("cmd");
  } else if (token.rfind("SCRIPT_", 0) == 0) {
    _handleScriptCommand(token, payload);
  }
}

void BLEConfigServer::_handleScriptCommand(const std::string &token, const std::string &payload) {
  const size_t maxLen = PSG_MAX_SCRIPT_CHARS;
  auto emitListenerStatus = [&](const String &msg){ if (_listener) _listener->onPatternScriptStatus(msg); };
  if (token == "SCRIPT_BEGIN") {
    if (payload.empty()) {
      notifyStatus("[SCRIPT] ERR begin missing length");
      emitListenerStatus("[SCRIPT] ERR begin missing length");
      return;
    }
    const char *buf = payload.c_str();
    char *endPtr = nullptr;
    long expected = strtol(buf, &endPtr, 10);
    while (endPtr && *endPtr != '\0' && isspace((unsigned char)*endPtr)) ++endPtr;
    int slot = -1;
    if (endPtr && *endPtr != '\0') {
      char *slotEnd = nullptr;
      slot = static_cast<int>(strtol(endPtr, &slotEnd, 10));
    }
    if (expected <= 0) {
      notifyStatus("[SCRIPT] ERR begin length");
      emitListenerStatus("[SCRIPT] ERR begin length");
      return;
    }
    if (expected > static_cast<long>(maxLen)) {
      notifyStatus(String("[SCRIPT] ERR too long len=") + String(expected));
      emitListenerStatus(String("[SCRIPT] ERR too long len=") + String(expected));
      return;
    }
    if (_scriptActive || _scriptReceivedLen > 0) {
      _resetScriptTransfer("preempt", false);
    }
    _scriptBuffer.clear();
    _scriptBuffer.reserve(static_cast<size_t>(expected));
    _scriptExpectedLen = static_cast<size_t>(expected);
    _scriptReceivedLen = 0;
    _scriptTargetSlot = slot;
    _scriptActive = true;
    _scriptLastChunkMs = millis();
    _scriptProgressDirty = true;
    _scriptLastProgressNotifyMs = millis();
    String msg = String("[SCRIPT] BEGIN len=") + String(expected) + " slot=" + String(slot);
    notifyStatus(msg);
    emitListenerStatus(msg);
    return;
  }
  if (token == "SCRIPT_ABORT") {
    if (_scriptActive || _scriptReceivedLen > 0) {
      _resetScriptTransfer("abort");
    } else {
      notifyStatus("[SCRIPT] WARN abort no-session");
      emitListenerStatus("[SCRIPT] WARN abort no-session");
    }
    return;
  }
  if (token == "SCRIPT_END") {
    notifyStatus("[SCRIPT] CMD END");
    if (!_scriptActive) {
      notifyStatus("[SCRIPT] ERR end without begin");
      emitListenerStatus("[SCRIPT] ERR end without begin");
      return;
    }
    emitListenerStatus("[SCRIPT] CMD END");
    _finalizeScriptTransfer();
    return;
  }
  if (token == "SCRIPT_STATUS") {
    String msg = String("[SCRIPT] STATE active=") + String(_scriptActive ? 1 : 0) +
                 " recv=" + String(static_cast<unsigned long>(_scriptReceivedLen)) +
                 " exp=" + String(static_cast<unsigned long>(_scriptExpectedLen));
    notifyStatus(msg);
    emitListenerStatus(msg);
    return;
  }
  notifyStatus(String("[SCRIPT] ERR unknown cmd ") + token.c_str());
  emitListenerStatus(String("[SCRIPT] ERR unknown cmd ") + token.c_str());
}

void BLEConfigServer::_resetScriptTransfer(const char *reasonTag, bool notify) {
  bool hadProgress = _scriptActive || _scriptReceivedLen > 0;
  if (notify && hadProgress) {
    String msg = String("[SCRIPT] RESET reason=") + (reasonTag ? reasonTag : "?");
    this->notifyStatus(msg);
    if (_listener) _listener->onPatternScriptStatus(msg);
  }
  _scriptBuffer.clear();
  _scriptExpectedLen = 0;
  _scriptReceivedLen = 0;
  _scriptTargetSlot = -1;
  _scriptActive = false;
  _scriptLastChunkMs = 0;
  _scriptProgressDirty = false;
  _scriptLastProgressNotifyMs = 0;
}

void BLEConfigServer::_finalizeScriptTransfer() {
  if (!_scriptActive) {
    notifyStatus("[SCRIPT] ERR finalize inactive");
    if (_listener) _listener->onPatternScriptStatus("[SCRIPT] ERR finalize inactive");
    return;
  }
  if (_scriptReceivedLen != _scriptExpectedLen) {
    String msg = String("[SCRIPT] ERR size mismatch recv=") +
                 String(static_cast<unsigned long>(_scriptReceivedLen)) +
                 " exp=" + String(static_cast<unsigned long>(_scriptExpectedLen));
    notifyStatus(msg);
    if (_listener) _listener->onPatternScriptStatus(msg);
    _resetScriptTransfer("size", false);
    return;
  }
  std::string script;
  script.swap(_scriptBuffer);
  size_t len = _scriptExpectedLen;
  int slot = _scriptTargetSlot;
  _scriptExpectedLen = 0;
  _scriptReceivedLen = 0;
  _scriptTargetSlot = -1;
  _scriptActive = false;
  _scriptLastChunkMs = 0;
  _scriptProgressDirty = false;
  _scriptLastProgressNotifyMs = 0;
  notifyStatus(String("[SCRIPT] FINALIZE len=") + String(static_cast<unsigned long>(len)));
  String msg = String("[SCRIPT] READY len=") + String(static_cast<unsigned long>(len));
  notifyStatus(msg);
  if (_listener) {
    _listener->onPatternScriptStatus(msg);
    _listener->onPatternScriptReceived(script, slot);
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

void BLEConfigServer::ScriptCallbacks::onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) {
  (void)info;
  const std::string chunk = c->getValue();
  size_t chunkLen = chunk.size();
  if (chunkLen == 0) {
    return;
  }
  if (!_parent->_scriptActive) {
    _parent->notifyStatus("[SCRIPT] WARN chunk without begin");
    if (_parent->_listener) _parent->_listener->onPatternScriptStatus("[SCRIPT] WARN chunk without begin");
    return;
  }
  if (_parent->_scriptReceivedLen + chunkLen > _parent->_scriptExpectedLen) {
    String msg = String("[SCRIPT] ERR overflow chunk=") + String(static_cast<unsigned long>(chunkLen));
    _parent->notifyStatus(msg);
    if (_parent->_listener) _parent->_listener->onPatternScriptStatus(msg);
    _parent->_resetScriptTransfer("overflow", false);
    return;
  }
  _parent->_scriptBuffer.append(chunk.data(), chunkLen);
  _parent->_scriptReceivedLen += chunkLen;
  _parent->_scriptLastChunkMs = millis();
  _parent->_scriptProgressDirty = true;
}
