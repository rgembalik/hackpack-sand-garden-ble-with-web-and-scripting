#include "BLEConfigServer.h"

BLEConfigServer::BLEConfigServer() {}

void BLEConfigServer::begin(ISGConfigListener *listener) {
  _listener = listener;
  NimBLEDevice::init(SG_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Max power (adjust if needed)

  _server = NimBLEDevice::createServer();
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
  adv->addServiceUUID(_service->getUUID());
  adv->setAppearance(0x0000);              // generic appearance
  NimBLEDevice::setDeviceName(SG_DEVICE_NAME); // reinforce name
  // Start advertising
  adv->start();
}

void BLEConfigServer::loop() {
  // Nothing needed yet. Placeholder for future (e.g., connection watchdog)
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
