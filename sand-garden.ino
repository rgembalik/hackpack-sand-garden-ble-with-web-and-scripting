/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (Sand Garden Code)

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ************************************************************************************
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
INCLUDED LIBRARIES
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <elapsedMillis.h> //Creates timer objects that are more convenient for non-blocking timing than millis()
#include <AccelStepper.h>  //Controls the stepper motors
#include <FastLED.h>         //Controls the RGB LEDs
#include <OneButtonTiny.h>   //Button management and debouncing
#include <NimBLEDevice.h>    // Ensure NimBLE core available for BLEConfigServer
#include "BLEConfigServer.h" // BLE configuration server (modular config service)
#include "Positions.h"       // shared Positions struct for external modules
#include "PatternScript.h"   // SandScript compiler/runtime

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
PREPROCESSOR DIRECTIVES.

Useful values and limits for defining how the sand garden will behave. In most cases, these values should not be changed.

*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// #define REVERSE_R_MOTOR               //uncomment this line to switch the direction the radial axis motor runs
// #define REVERSE_A_MOTOR               //uncomment this line to switch the direction the angular axis motor runs

#define STEPS_PER_MOTOR_REV 2048                     // Number of motor steps in one revolution of the output shaft of the motor.
#define STEPS_PER_A_AXIS_REV 2 * STEPS_PER_MOTOR_REV // the number of steps required to move the angular axis one full revolution
#define TRAVEL_PER_PINION_REV 50.267                 // Distance in mm the rack moves in one complete revolution of the pinion. (Informational only)
#define STEPS_PER_MM 70.0f                          // Calibrated motor steps per mm of radial travel (7000 steps span ≈ 10 cm).
#define MM_PER_STEP (1.0f / STEPS_PER_MM)           // Millimeters of travel per motor step on radial axis (≈0.01429 mm with current mapping).
#define STEPS_PER_DEG (STEPS_PER_A_AXIS_REV) / 360  // Motor steps per degree of motion on angular axis. Should be about 11.378 steps per degree.
#define STEPS_PER_RAD STEPS_PER_MOTOR_REV / PI      // Motor steps per radian of motion on angular axis. About 652.

#define ACTUAL_LEN_R_MM 102.3857f                       // Effective radial travel (mm) including soft-stop margin for the 10 cm span.
#define ACTUAL_LEN_R_STEPS ACTUAL_LEN_R_MM * STEPS_PER_MM // Maximum possible length of radius in motor steps. ≈7167 with current mapping.
#define MAX_R_STEPS 7000                                 // Soft limit on how far the radius can move in terms of steps of the motor. This leaves a slight buffer on each end.
// (Removed unused MAX_R_MM macro)

#define HOMING_BUFFER (ACTUAL_LEN_R_STEPS - MAX_R_STEPS) / 2 // Crash home R axis to 0, then move this many steps in positive direction to create a soft stop.
#define RELAXATION_BUFFER STEPS_PER_DEG * 5                  // Crash homing tensions the bead chain, and backlash and flex in the gantry need to be released.

#define MAX_SPEED_R_MOTOR 550.0 // Maximum speed in steps per second for radius motor. Faster than this is unreliable.
#define MAX_SPEED_A_MOTOR 550.0 // Maximum speed in steps per second for angle motor.

// Angular speed reduction now computed dynamically inside moveToPosition() using current max speed values.

// Pin definitions follow.
// The #ifdef / #endif blocks are used to check to see if either REVERSE_R_MOTOR or REVERSE_A_MOTOR
// is defined at the very top of the code, and if they are, the order the pins are defined in changes.

#ifdef REVERSE_A_MOTOR
#define MOTORA_IN1_PIN 12
#define MOTORA_IN2_PIN 11
#define MOTORA_IN3_PIN 10
#define MOTORA_IN4_PIN 9
#endif

#ifndef REVERSE_A_MOTOR
#define MOTORA_IN1_PIN 9
#define MOTORA_IN2_PIN 10
#define MOTORA_IN3_PIN 11
#define MOTORA_IN4_PIN 12
#endif

#ifdef REVERSE_R_MOTOR
#define MOTORR_IN1_PIN 5
#define MOTORR_IN2_PIN 6
#define MOTORR_IN3_PIN 7
#define MOTORR_IN4_PIN 8
#endif

#ifndef REVERSE_R_MOTOR
#define MOTORR_IN1_PIN 8 // The motor is flipped upside down in assembly, so pin order is reversed from other motor.
#define MOTORR_IN2_PIN 7
#define MOTORR_IN3_PIN 6
#define MOTORR_IN4_PIN 5
#endif

#define JOYSTICK_A_PIN A2       // Left-right axis of joystick, associated with changing angular axis in manual mode
#define JOYSTICK_R_PIN A3       // Up-down axis of joystick, associated with changing radial axis in manual mode
#define BUTTON_PIN A1           // Joystick button pin
#define RANDOM_SEED_PIN A6      // used to generate random numbers.
#define LED_DATA_PIN GPIO_NUM_1 // The output for the LED bar.
#define NUM_LEDS 8              // Number of LEDs in the bar.
#define MAX_BRIGHTNESS 40       // Brightness values are 8-bit for a max of 255 (the range is [0-255]), this sets default maximum to 40 out of 255.
#define LED_FADE_PERIOD 1000    // Amount of time in milliseconds it takes for LEDs to fade on and off.

// Struct used for storing positions of the axes, as well as storing the values of the joystick.
// Positions struct definition moved to Positions.h for sharing with PatternScript and future modules.

// Joystick manual mode threshold retained; removed unused runtime calibration scaffolding.
static const int JOYSTICK_MANUAL_MOVE_THRESHOLD = 12; // require this magnitude before commanding motion steps

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MOTION CONTROL AND PATTERN GENERATION

The following items are for tracking the position of the gantry, reading the joystick, and defining target positions for
the gantry through the use of pattern functions.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Create two objects, one for each stepper motor.
AccelStepper stepperAngle(4, MOTORA_IN1_PIN, MOTORA_IN3_PIN, MOTORA_IN2_PIN, MOTORA_IN4_PIN);  // angular axis motor
AccelStepper stepperRadius(4, MOTORR_IN1_PIN, MOTORR_IN3_PIN, MOTORR_IN2_PIN, MOTORR_IN4_PIN); // radial axis motor

// (Positions struct moved above for early visibility)

// These variables of type Positions (defined above) are for storing gantry positions and joystick values
Positions currentPositions; // store the current positions of the axes in this
Positions targetPositions;  // store the desired positions of the axes in this
Positions joystickValues;   // store the potentiometer values of the joystick in this. I didn't want to make a new struct just for the joystick.

// Function prototypes for pattern generators. Each pattern function has to return a struct of type Positions.
// This will be used as the target position for the motion controller. Note that these are just
// function prototypes. They are put up here to let the compiler know that they will be defined later in the code.
Positions pattern_SimpleSpiral(Positions current, bool restartPattern = false);        // Simple spiral. Grows outward, then inward.
Positions pattern_Cardioids(Positions current, bool restartPattern = false);           // Cardioids
Positions pattern_WavySpiral(Positions current, bool restartPattern = false);          // Wavy spiral.
Positions pattern_RotatingSquares(Positions current, bool restartPattern = false);     // Rotating squares
Positions pattern_PentagonSpiral(Positions current, bool restartPattern = false);      // Pentagon spiral
Positions pattern_HexagonVortex(Positions current, bool restartPattern = false);       // Hexagon vortex
Positions pattern_PentagonRainbow(Positions current, bool restartPattern = false);     // Pentagon rainbow
Positions pattern_RandomWalk1(Positions current, bool restartPattern = false);         // Random walk 1 (connected by arcs)
Positions pattern_RandomWalk2(Positions current, bool restartPattern = false);         // Random walk 2 (connected by lines)
Positions pattern_AccidentalButterfly(Positions current, bool restartPattern = false); // Accidental Butterfly
Positions pattern_SandScript(Positions current, bool restartPattern = false);          // SandScript-driven pattern (dynamic slot)

/**
 * @brief Typedef for storing pointers to pattern-generating functions.
 *
 * This typedef defines a custom data type PatternFunction for storing pointers to pattern functions.
 * It allows pattern functions to be called by passing the appropriate index number to an array of pattern function pointers,
 * simplifying the process of switching between patterns. Each pattern function takes a Positions struct and a bool as parameters
 * and returns the next target position as a Positions struct.
 *
 * @typedef PatternFunction
 *
 * This typedef enables pattern switching by indexing into an array of pattern functions, making it easy to select and execute
 * different patterns dynamically.
 */
typedef Positions (*PatternFunction)(Positions, bool);

/**
 * @brief Array of pattern-generating functions.
 *
 * This array stores the functions responsible for generating different patterns, defined using the PatternFunction typedef.
 * To add a new pattern function, follow these steps:
 * 1. Declare the new pattern function prototype (e.g., Positions pattern_42(Positions current);).
 * 2. Add the new pattern function to this array.
 * 3. Define the function at the end of the code.
 *
 * @note The array is 0-indexed, but the controller interface (joystick and LEDs) uses 1-indexing.
 * Therefore, pattern 1 is stored at index 0, pattern 2 at index 1, and so on. This offset is handled within the code,
 * but keep it in mind when working with the array.
 */
PatternFunction patterns[] = {pattern_SimpleSpiral, pattern_Cardioids, pattern_WavySpiral, pattern_RotatingSquares, pattern_PentagonSpiral, pattern_HexagonVortex, pattern_PentagonRainbow, pattern_RandomWalk1,
                              pattern_RandomWalk2, pattern_AccidentalButterfly, pattern_SandScript};

static const size_t PATTERN_COUNT = sizeof(patterns) / sizeof(patterns[0]);
static const int SCRIPT_PATTERN_INDEX = static_cast<int>(PATTERN_COUNT); // 1-based index for BLE/UI

struct ActiveSandScript {
  PatternScript program;
  PatternScriptRuntime runtime;
  bool loaded = false;
  String label = String("Dynamic");     // descriptive tag for status logs
  String lastError; // most recent compile/runtime error message
  String source;    // raw source (for diagnostics/status echo)
};

static ActiveSandScript g_activeScript;
static const uint32_t SANDSCRIPT_MIN_STEP_INTERVAL_US = 4000; // ~250 Hz evaluation ceiling
static const uint32_t SANDSCRIPT_IDLE_EVAL_INTERVAL_US = 40000; // 25 Hz when fully idle
static const uint16_t SANDSCRIPT_IDLE_SPIN_THRESHOLD = 250;     // ~1 second of no motion at 4 ms cadence
static const uint16_t SANDSCRIPT_IDLE_STEP_TOLERANCE_STEPS = 2; // treat <=2 step delta as zero motion
static elapsedMicros g_sandScriptStepTimer;
static uint16_t g_sandScriptIdleSpinCount = 0;
static bool g_sandScriptIdleBackoffActive = false;
static bool g_sandScriptIdleNotified = false;

struct SandScriptEvalInputsSnapshot
{
  float radiusCm = 0.0f;
  float angleDeg = 0.0f;
  float rev = 0.0f;
  uint32_t steps = 0;
  uint32_t timeMs = 0;
  bool start = false;
  bool valid = false;
};

static SandScriptEvalInputsSnapshot g_lastSandScriptEvalInputs;

static void resetActiveScriptRuntime() {
  g_activeScript.runtime = PatternScriptRuntime();
  g_sandScriptStepTimer = SANDSCRIPT_MIN_STEP_INTERVAL_US;
  g_sandScriptIdleSpinCount = 0;
  g_sandScriptIdleBackoffActive = false;
  g_sandScriptIdleNotified = false;
  g_lastSandScriptEvalInputs = SandScriptEvalInputsSnapshot();
}

static void clearActiveScript(const String &reason) {
  g_activeScript = ActiveSandScript();
  g_activeScript.label = "Dynamic";
  if (reason.length()) {
    g_activeScript.lastError = reason;
  }
  g_sandScriptStepTimer = SANDSCRIPT_MIN_STEP_INTERVAL_US;
  g_sandScriptIdleSpinCount = 0;
  g_sandScriptIdleBackoffActive = false;
  g_sandScriptIdleNotified = false;
  g_lastSandScriptEvalInputs = SandScriptEvalInputsSnapshot();
}

static bool compileAndActivateSandScript(const char *src, size_t len, const String &label, String &errOut) {
  if (!src) {
    errOut = "null script";
    clearActiveScript(errOut);
    return false;
  }
  if (len > PSG_MAX_SCRIPT_CHARS) {
    errOut = String("script exceeds ") + PSG_MAX_SCRIPT_CHARS + " chars";
    clearActiveScript(errOut);
    return false;
  }

  PatternScript compiled = {};
  String compileErr;
  PSGCompileResult res = compilePatternScript(src, compiled, &compileErr);
  if (res != PSG_OK) {
    errOut = compileErr.length() ? compileErr : String(psgErrorToString(res));
    clearActiveScript(errOut);
    return false;
  }

  g_activeScript.program = compiled;
  g_activeScript.loaded = true;
  g_activeScript.label = label.length() ? label : String("Dynamic");
  g_activeScript.lastError = "";
  g_activeScript.source = String(src);
  resetActiveScriptRuntime();
  errOut = "";
  return true;
}

static bool compileAndActivateSandScript(const std::string &src, const String &label, String &errOut) {
  return compileAndActivateSandScript(src.c_str(), src.size(), label, errOut);
}

struct SandScriptPreset {
  const char *name;
  const char *source;
};

static const SandScriptPreset kSandScriptPresets[] = {
  {
    "PingPongSharpPetals",
    "# simple back and forth with the ball,\n"
    "# no easing at the edge\n\n"
    "# sign(-10...10)\n"
    "r = pingpong(steps, 9)\n\n"
    "delta_angle = 3\n"
    "next_radius = 1 + r\n"
  },
  {
    "PingPongSoftPetals",
    "# simple back and forth with the ball \n"
    "# with sinusoidal easing\n\n"
    "# change direction every 15 steps\n"
    "dir = sin(steps * 180 / 15)\n\n"
    "delta_angle = 5\n"
    "delta_radius = 1 * dir\n"
  }
};

static const size_t SANDSCRIPT_PRESET_COUNT = sizeof(kSandScriptPresets) / sizeof(kSandScriptPresets[0]);

static int findSandScriptPresetIndex(const String &name) {
  String needle = name;
  needle.toLowerCase();
  for (size_t i = 0; i < SANDSCRIPT_PRESET_COUNT; ++i)
  {
    String candidate(kSandScriptPresets[i].name);
    candidate.toLowerCase();
    if (candidate == needle)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

static bool compileSandScriptPreset(size_t index, String &errOut) {
  if (index >= SANDSCRIPT_PRESET_COUNT)
  {
    errOut = "preset out of range";
    return false;
  }
  const SandScriptPreset &preset = kSandScriptPresets[index];
  return compileAndActivateSandScript(preset.source, strlen(preset.source), String(preset.name), errOut);
}

// ---------------- BLE Configuration Integration (listener defined later to access globals) ----------------
static BLEConfigServer bleConfig; // forward object; listener class will be defined after globals

static std::string g_pendingScriptSource;
static int g_pendingScriptSlot = -1;
static String g_pendingScriptLabel;
static volatile bool g_pendingScriptReady = false;
static uint32_t g_pendingScriptQueuedMs = 0;

static bool g_debugStepStreaming = false;
static uint32_t g_debugStepCounter = 0;

// Now that bleConfig is defined we implement the calibration function which emits a status line.

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
STATE MACHINE FLAGS:
This code uses simple state machine to keep track of which mode the machine is in (e.g., actively running a pattern, or in pattern selection mode).
These flags are used in that state machine.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int currentPattern = 1;           // default to pattern 1.
bool runPattern = false;          // this will be the start/stop flag. true means run the selected pattern.
bool buttonLongPressed = false;   // for indicating long press
bool autoMode = true;             // tracking if we're in automatic or manual mode. Defaults to auto on startup. If you want to start in manual drawing mode, set this to false.
bool motorsEnabled = true;        // used to track if motor drivers are enabled/disabled. initializes to enabled so the homing sequence can run.
bool patternSwitched = false;     // used for properly starting patterns from beginning when a new pattern is selected
int lastPattern = currentPattern; // used with currentPattern to detect pattern switching and set the patternSwitched flag.

// Telemetry cache
struct TelemetryCache
{
  int joyA = 0;
  int joyR = 0;
  int lastSentJoyA = 999; // force first send
  int lastSentJoyR = 999;
  uint8_t pattern = 0;
  bool autoMode = false;
  bool run = false;
  uint8_t brightness = 0;
};
static TelemetryCache telemetry;
static elapsedMillis telemetryHeartbeat;
static const unsigned long TELEMETRY_HEARTBEAT_PERIOD = 15000UL;
static const uint8_t JOY_DELTA_THRESHOLD = 5;

// Forward declarations for local-state wrappers
static void setPatternLocal(int p);
static void setAutoModeLocal(bool m);
static void setRunLocal(bool r);

// Listener implementation responding to remote config writes (now globals are defined)
class SandGardenConfigListener : public ISGConfigListener
{
public:
  void onSpeedMultiplierChanged(float newValue) override
  {
    float m = constrain(newValue, 0.1f, 3.0f);
    stepperAngle.setMaxSpeed(MAX_SPEED_A_MOTOR * m);
    stepperRadius.setMaxSpeed(MAX_SPEED_R_MOTOR * m);
    
  }
  void onCurrentPatternChanged(int newPattern) override
  {
    int total = sizeof(patterns) / sizeof(patterns[0]);
    if (newPattern >= 1 && newPattern <= total)
    {
      currentPattern = newPattern;
      patternSwitched = true; // force restart next loop
    }
  }
  void onAutoModeChanged(bool newAutoMode) override
  {
    if (autoMode != newAutoMode)
    {
      autoMode = newAutoMode;
      // Entering manual mode stops pattern run for safety unless already stopped
      if (!autoMode)
      {
        runPattern = false;
      }
      else
      {
        // In auto mode if run state previously requested start running
        if (runPattern)
          patternSwitched = true; // ensure pattern restarts cleanly
      }
    }
  }
  void onRunStateChanged(bool newRunState) override
  {
    if (runPattern != newRunState)
    {
      runPattern = newRunState;
      if (runPattern)
      {
        patternSwitched = true; // restart pattern when starting
      }
    }
  }
  void onPatternScriptStatus(const String &msg) override
  {
    bleConfig.notifyStatus(msg);
  }
  void onPatternScriptReceived(const std::string &script, int slotIndex) override
  {
    String label = String("BLE");
    if (slotIndex >= 1)
    {
      label += String("-slot") + String(slotIndex);
    }
    bool replacing = g_pendingScriptReady;
    g_pendingScriptSource = script;
    g_pendingScriptSlot = slotIndex;
    g_pendingScriptLabel = label;
    g_pendingScriptReady = true;
    g_pendingScriptQueuedMs = millis();
    String msg = String("[SCRIPT] QUEUED len=") + String((unsigned long)script.size());
    if (replacing)
    {
      msg += " (replacing previous pending)";
    }
    bleConfig.notifyStatus(msg);
  }
  void onCommandReceived(const String &cmd, const std::string &raw) override
  {
    if (cmd == "SCRIPT_PRESET")
    {
      String rawStr(raw.c_str());
      int space = rawStr.indexOf(' ');
      String arg = "";
      if (space >= 0)
      {
        arg = rawStr.substring(space + 1);
        arg.trim();
      }
      if (arg.equalsIgnoreCase("list"))
      {
        String msg = "[SCRIPT] PRESETS";
        for (size_t i = 0; i < SANDSCRIPT_PRESET_COUNT; ++i)
        {
          msg += " ";
          msg += String(i + 1);
          msg += ":";
          msg += kSandScriptPresets[i].name;
        }
        bleConfig.notifyStatus(msg);
        return;
      }

      int presetIndex = 0;
      bool presetResolved = false;
      if (arg.length() > 0)
      {
        bool argIsNumber = true;
        for (size_t i = 0; i < (size_t)arg.length(); ++i)
        {
          if (!isDigit(arg[i]))
          {
            argIsNumber = false;
            break;
          }
        }
        if (argIsNumber)
        {
          long val = arg.toInt();
          if (val < 1)
            val = 1;
          presetIndex = static_cast<int>(val - 1);
          presetResolved = (presetIndex >= 0 && presetIndex < (int)SANDSCRIPT_PRESET_COUNT);
        }
        else
        {
          int found = findSandScriptPresetIndex(arg);
          if (found >= 0)
          {
            presetIndex = found;
            presetResolved = true;
          }
        }
      }
      else
      {
        presetResolved = true; // default to first preset
        presetIndex = 0;
      }

      if (!presetResolved)
      {
        bleConfig.notifyStatus(String("[SCRIPT] PRESET_ERR unknown ") + arg);
        return;
      }

      String err;
      if (compileSandScriptPreset(presetIndex, err))
      {
        String msg = String("[SCRIPT] PRESET_LOAD idx=") + String(presetIndex + 1) + " name=" + kSandScriptPresets[presetIndex].name;
        bleConfig.notifyStatus(msg);
        if (currentPattern == SCRIPT_PATTERN_INDEX)
        {
          patternSwitched = true;
        }
        else
        {
          setPatternLocal(SCRIPT_PATTERN_INDEX);
        }
      }
      else
      {
        if (!err.length())
        {
          err = g_activeScript.lastError;
        }
        bleConfig.notifyStatus(String("[SCRIPT] PRESET_ERR ") + err);
      }
      return;
    }

    if (cmd == "DEBUG_STEPS")
    {
      String rawStr(raw.c_str());
      String arg;
      int space = rawStr.indexOf(' ');
      if (space >= 0)
      {
        arg = rawStr.substring(space + 1);
        arg.trim();
      }

      if (!arg.length())
      {
        bleConfig.notifyStatus(String("[DEBUG] STEPS ") + (g_debugStepStreaming ? "ON" : "OFF") + " count=" + String((unsigned long)g_debugStepCounter));
        return;
      }

      bool newState = g_debugStepStreaming;
      if (arg.equalsIgnoreCase("ON") || arg.equalsIgnoreCase("TRUE") || arg == "1")
      {
        newState = true;
      }
      else if (arg.equalsIgnoreCase("OFF") || arg.equalsIgnoreCase("FALSE") || arg == "0")
      {
        newState = false;
      }
      else if (arg.equalsIgnoreCase("TOGGLE"))
      {
        newState = !g_debugStepStreaming;
      }
      else
      {
        bleConfig.notifyStatus(String("[DEBUG] STEPS_ERR arg=") + arg);
        return;
      }

      g_debugStepStreaming = newState;
      g_debugStepCounter = 0;
      bleConfig.notifyStatus(String("[DEBUG] STEPS ") + (newState ? "ON" : "OFF"));
      return;
    }

    if (cmd.startsWith("SCRIPT_"))
    {
      return; // handled by BLEConfigServer internally
    }
    // Unknown command
    bleConfig.notifyStatus(String("[CMD] UNKNOWN ") + cmd);
  }
};
static SandGardenConfigListener bleListener;

static void processPendingSandScript()
{
  if (!g_pendingScriptReady)
  {
    return;
  }

  std::string script = std::move(g_pendingScriptSource);
  g_pendingScriptSource.clear();
  int slot = g_pendingScriptSlot;
  g_pendingScriptSlot = -1;
  String label = g_pendingScriptLabel;
  g_pendingScriptLabel = String();
  g_pendingScriptReady = false;
  uint32_t queuedAt = g_pendingScriptQueuedMs;
  g_pendingScriptQueuedMs = 0;

  uint32_t now = millis();
  uint32_t waitMs = queuedAt ? (now - queuedAt) : 0;
  String startMsg = String("[SCRIPT] COMPILE start bytes=") + String((unsigned long)script.size());
  if (waitMs > 0)
  {
    startMsg += " wait=";
    startMsg += String((unsigned long)waitMs);
    startMsg += "ms";
  }
  bleConfig.notifyStatus(startMsg);

  yield();

  String err;
  bool ok = compileAndActivateSandScript(script, label, err);
  if (ok)
  {
    bleConfig.notifyStatus("[SCRIPT] COMPILE ok");
    int resolvedSlot = (slot >= 1 && slot <= (int)PATTERN_COUNT) ? slot : SCRIPT_PATTERN_INDEX;
    String msg = String("[SCRIPT] LOADED bytes=") + String((unsigned long)script.size()) + " slot=" + String(resolvedSlot);
    if (g_activeScript.label.length())
    {
      msg += " tag=" + g_activeScript.label;
    }
    bleConfig.notifyStatus(msg);
    if (currentPattern == resolvedSlot)
    {
      patternSwitched = true;
    }
    else
    {
      setPatternLocal(resolvedSlot);
    }
  }
  else
  {
    String msg = String("[SCRIPT] COMPILE_ERR ") + err;
    bleConfig.notifyStatus(msg);
    if (!err.length())
    {
      bleConfig.notifyStatus(String("[SCRIPT] CODE ") + psgErrorToString(PSG_ERR_SYNTAX));
    }
  }
}

// Wrapper functions ensure BLE characteristics stay authoritative when local inputs change
static void setPatternLocal(int p)
{
  int total = sizeof(patterns) / sizeof(patterns[0]);
  if (p < 1)
    p = 1;
  if (p > total)
    p = total;
  if (p == currentPattern)
    return;
  bleConfig.setCurrentPattern(p); // triggers listener which updates currentPattern & patternSwitched
  bleConfig.notifyStatus("[PATTERN] id=" + String(p));
}

static void setAutoModeLocal(bool m)
{
  if (m == autoMode)
    return;
  bleConfig.setAutoMode(m); // listener handles side-effects
  bleConfig.notifyStatus(String("[MODE] mode=") + (m ? "AUTO" : "MANUAL"));
}

static void setRunLocal(bool r)
{
  if (r == runPattern)
    return;
  bleConfig.setRunState(r); // listener updates runPattern
  bleConfig.notifyStatus(String("[RUN] state=") + (r ? "STARTED" : "STOPPED"));
}

static void sendStateTelemetry()
{
  // Aggregate snapshot
  String msg = "STATE pat=" + String(currentPattern) +
               " auto=" + String(autoMode ? 1 : 0) +
               " run=" + String(runPattern ? 1 : 0) +
               " brt=" + String(FastLED.getBrightness());
  bleConfig.notifyTelemetry(msg);
}

static void heartbeatTelemetry()
{
  if (telemetryHeartbeat >= TELEMETRY_HEARTBEAT_PERIOD)
  {
    telemetryHeartbeat = 0;
    bleConfig.notifyTelemetry("HB");
  }
}

// Forward declarations for conversion helpers used by telemetry routines
float convertStepsToDegrees(int steps);
float convertStepsToRadians(float steps);
float convertStepsToMM(float steps);

static void emitPatternStepTelemetry(const Positions &planned, const Positions &actual, bool restartFlag)
{
  if (!g_debugStepStreaming)
  {
    return;
  }

  uint32_t idx = ++g_debugStepCounter;
  const float radiusMm = convertStepsToMM(static_cast<float>(actual.radial));
  const float angleDeg = convertStepsToDegrees(actual.angular);
  const float angleRad = convertStepsToRadians(static_cast<float>(actual.angular));
  const float xMm = radiusMm * cosf(angleRad);
  const float yMm = radiusMm * sinf(angleRad);

  String msg = String("STEP idx=") + String(idx) +
               " tr=" + String(planned.radial) +
               " ta=" + String(planned.angular) +
               " cr=" + String(actual.radial) +
               " ca=" + String(actual.angular) +
               " mm=" + String(radiusMm, 2) +
               " deg=" + String(angleDeg, 2) +
               " x=" + String(xMm, 2) +
               " y=" + String(yMm, 2) +
               " pat=" + String(currentPattern) +
               " auto=" + String(autoMode ? 1 : 0) +
               " run=" + String(runPattern ? 1 : 0) +
               " start=" + String(restartFlag ? 1 : 0);

  if (currentPattern == SCRIPT_PATTERN_INDEX && g_lastSandScriptEvalInputs.valid)
  {
    msg += " dslrcm=" + String(g_lastSandScriptEvalInputs.radiusCm, 3);
    msg += " dsladeg=" + String(g_lastSandScriptEvalInputs.angleDeg, 2);
    msg += " dslrev=" + String(g_lastSandScriptEvalInputs.rev, 4);
    msg += " dslsteps=" + String((unsigned long)g_lastSandScriptEvalInputs.steps);
    msg += " dsltime=" + String((unsigned long)g_lastSandScriptEvalInputs.timeMs);
    msg += " dslstart=" + String(g_lastSandScriptEvalInputs.start ? 1 : 0);
  }

  if (currentPattern == SCRIPT_PATTERN_INDEX && g_activeScript.label.length())
  {
    msg += " tag=";
    msg += g_activeScript.label;
  }

  bleConfig.notifyTelemetry(msg);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
FUNCTION PROTOTYPES.

These are basically a way of telling the compiler that we will have functions with these names and parameters, which will be defined later in the code.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Math related functions
long convertDegreesToSteps(float degrees);                                     // for converting degrees to motor steps
float convertStepsToDegrees(int steps);                                        // for converting motor steps to degrees
long convertRadiansToSteps(float rads);                                        // For converting radians to steps on angular axis
float convertStepsToRadians(float steps);                                      // For converting steps to radians on angular axis
int convertMMToSteps(float mm);                                                // for converting millimeters to steps on radial axis
float convertStepsToMM(float steps);                                           // for converting steps to millimeters on the radial axis
float fmap(float n, float in_min, float in_max, float out_min, float out_max); // version of map() that works for floating point numbers
int modulus(int x, int y);                                                     // Use for wrapping values around at ends of range. like %, but no negative numbers.

// Movement related functions
int findShortestPathToPosition(int current, int target, int wrapValue);             // For finding the shortest path to the new position on the angular axis
int calcRadialChange(int angularMoveInSteps, int radialMoveInSteps);                // for figuring out the relative change on the radial axis
int calcRadialSteps(int current, int target, int angularOffsetSteps);               // For calculating actual number of steps radial motor needs to take.
// (Removed unused calculateDistanceBetweenPoints prototype)
void homeRadius();                                                                  // for homing the radial axis on startup
void moveToPosition(long angularSteps, long radialSteps);                           // for moving both axes to target position simultaneously.
Positions orchestrateMotion(Positions currentPositions, Positions targetPositions); // Encapsulates steps required to move to target position and returns the new current position.

// Miscellaneous functions
Positions readJoystick(void); // returns a struct containing the current joystick values

// Geometry generation functions
Positions drawLine(Positions point0, Positions point1, Positions current, int resolution, bool reset);                // For drawing a straight line between two points
void nGonGenerator(Positions *pointArray, int numPoints, Positions centerPoint, int radius, float rotationDeg = 0.0); // generates a list of points that form a polygon's vertices
void translatePoints(Positions *pointArray, int numPoints, Positions translationVector);                              // For moving an array of points along a vector to a new position
// Removed unused geometry transformation prototypes (scalePoints, rotatePoints, reflectPoints)

#pragma region LedDisplayClass

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This is a class that contains all the functions and data required to handle the LED display bar.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class LedDisplay
{
private:
  CRGB leds[NUM_LEDS]; // array that holds the state of each LED

public:
  // This is the constructor. It's called when a new instance of the class is created, and handles setting things up for use.
  LedDisplay()
  {
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(MAX_BRIGHTNESS);
    FastLED.clear();
    // Small stabilization delay; helps some ESP32 targets before first show
    delay(5);
#if SG_FASTLED_DIAG_BOOT
    for (int i = 0; i < NUM_LEDS; i++)
      leds[i] = CRGB::Blue;
    FastLED.show();
    delay(150);
    for (int i = 0; i < NUM_LEDS; i++)
      leds[i] = CRGB::Black;
    FastLED.show();
#endif
  }

  // a proxy function for setting the brightness of the LEDs. This way the class can handle all the LED stuff
  // without relying on the user to sometimes call on FastLED directly.
  void setBrightness(uint8_t val)
  {
    FastLED.setBrightness(val);
  }

  /**
   * @brief Indicates the currently selected pattern by lighting up LEDs in a specific color.
   *
   * This function uses the FastLED library to light up the LEDs in a specific pattern to indicate which pattern is selected.
   * A solid color is shown to indicate that the machine is in pattern selection mode. If the value is 255, the manual drawing
   * mode is indicated by lighting a single LED in DarkCyan. For other pattern values, the LEDs are lit using bitwise operations
   * to determine which LEDs should be turned on.
   *
   * @param value The current pattern number, where 255 indicates manual drawing mode, and other values indicate specific patterns.
   *
   * @note The .fadePixels() method can be used to make the LEDs fade, indicating that the machine is running a pattern. This function
   * uses bitwise operations to determine the LED pattern, lighting the LEDs in MediumVioletRed for non-manual patterns.
   */
  void indicatePattern(uint8_t value)
  { // used for showing which pattern is selected
    FastLED.clear();
    if (value == 255)
    { // pattern255 is the manual drawing mode.
      FastLED.clear();
      leds[0] = CRGB::DarkCyan;
    }
    else
    { // all other patterns can be displayed with bitwise operations
      for (int i = 0; i < NUM_LEDS; i++)
      { // iterate through each LED in the array
        if (value & (1 << i))
        {                                                 // bitwise AND the value of each bit in the pattern number to determine if current LED needs to be turned on.
          leds[NUM_LEDS - 1 - i] = CRGB::MediumVioletRed; // turn on the LED if needed
        }
      }
    }
    FastLED.show(); // display the LEDs
  }

  /**
   * @brief Gradually fades the LEDs on and off over time to indicate that a pattern is running.
   *
   * This function automatically controls the brightness of the LEDs, causing them to fade in and out over a specified period.
   * It is intended to be used when the machine is running a pattern to provide a visual indication of operation.
   *
   * @param period The time in milliseconds it takes for the LEDs to fade in and out (complete cycle).
   * @param maxBrightness The maximum brightness level the LEDs will reach during the fade cycle.
   *
   * The function calculates the current brightness based on the time position in the fade cycle, applying the appropriate brightness
   * to all LEDs using the FastLED.setBrightness() function.
   */
  void fadePixels(unsigned long period, uint8_t maxBrightness)
  {
    unsigned long currentTime = millis();
    unsigned long timeInCycle = currentTime % period; // Time position in current cycle
    unsigned long halfPeriod = period / 2;
    int brightness;

    // Determine phase and calculate brightness
    if (timeInCycle < halfPeriod)
    {
      // Fading in
      brightness = map(timeInCycle, 0, halfPeriod, 0, maxBrightness);
    }
    else
    {
      // Fading out
      brightness = map(timeInCycle, halfPeriod, period, maxBrightness, 0);
    }

    // Apply calculated brightness to all LEDs
    FastLED.setBrightness(brightness);
    FastLED.show();
  }

  /**
   * @brief Animates an LED bouncing pattern during the homing process and flashes green when homing is complete.
   *
   * This function animates a bouncing light pattern on the LEDs to indicate that the gantry is in the process of homing.
   * Once homing is complete, the LEDs flash green to signal completion. The function can block execution briefly during the
   * flashing portion after homing is done.
   *
   * @param homingComplete A boolean flag indicating whether the homing process is complete. If set to false, the animation continues.
   * If set to true, the LEDs flash green to indicate completion.
   *
   * The animation consists of a bouncing light pattern with a color that changes over time. When the gantry finishes homing,
   * the LEDs flash green in a blocking manner for a brief period.
   */
  void homingSequence(bool homingComplete = false)
  {
    static unsigned long lastUpdate = 0;

    const byte fadeAmount = 150;
    const int ballWidth = 2;
    const int deltaHue = 4;

    static byte hue = HUE_RED;
    static int direction = 1;
    static int position = 0;
    static int multiplier = 1;

    FastLED.setBrightness(MAX_BRIGHTNESS);

    if (!homingComplete)
    { // If the homing sequence is not complete, animate this pattern.
      if (millis() - lastUpdate >= 100)
      {
        hue += deltaHue;
        position += direction;

        if (position == (NUM_LEDS - ballWidth) || position == 0)
          direction *= -1;

        for (int i = 0; i < ballWidth; i++)
        {
          leds[position + i].setHue(hue);
        }

        // Randomly fade the LEDs
        for (int j = 0; j < NUM_LEDS; j++)
        {
          // if (random(10) > 3)
          leds[j] = leds[j].fadeToBlackBy(fadeAmount);
        }
        FastLED.show();
        lastUpdate = millis();
      }
    }
    else
    { // if the homing sequence is complete, indicate that by flashing the LEDs briefly.
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB::Green;
      }

      for (int j = 0; j < 8; j++)
      {
        FastLED.setBrightness(constrain(MAX_BRIGHTNESS * multiplier, 0, MAX_BRIGHTNESS));
        multiplier *= -1;
        FastLED.show();
        delay(100);
      }
    }
  }
};

#pragma endregion LedDisplayClass

// Create an instance of the LedDisplay class that controls the RGB LEDs.
LedDisplay display;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MISC. GLOBAL VARIABLES.
Used for tracking time and button presses.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

elapsedMillis lastJoystickUpdate; // used to track the last time the joystick was updated to prevent absurdly fast scrolling

// Create an object that handles the joystick button
OneButtonTiny button(BUTTON_PIN, true, true); // set up the button (button pin, active low, enable internal pull-up resistor)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
SETUP FUNCTION (runs once when Arduino powers on)
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // Generate a random seed. If you want to use pseudorandom numbers in a pattern, this makes them more random.
  // Make sure that RANDOM_SEED_PIN is an analog pin that's not connected to anything.
  randomSeed(analogRead(RANDOM_SEED_PIN));
  // configure the joystick and button pins
  pinMode(JOYSTICK_A_PIN, INPUT);
  pinMode(JOYSTICK_R_PIN, INPUT);

  // Perform runtime joystick calibration (samples for ~700ms, then tightens deadzone)
  // Set up the button.
  // Single press of button is for starting or stopping the current pattern.
  button.attachClick([]() {   // This is called a lambda function. Basically it's a nameless function that runs when the button is single pressed.
    setRunLocal(!runPattern); // route via BLE sync wrapper
    bleConfig.notifyStatus("[BTN] state=CLICK");
  });

  // Attaching an event to the long press of the button. Currently, long pressing the button lets you end the homing process early.
  button.attachLongPressStart([]()
                              {
    buttonLongPressed = true;         
    bleConfig.notifyStatus("[BTN] state=LONGPRESS"); });

  // set the maximum speeds and accelerations for the stepper motors.
  stepperAngle.setMaxSpeed(MAX_SPEED_A_MOTOR);
  stepperAngle.setAcceleration(5000.0); // Need high acceleration without losing steps.
  stepperRadius.setMaxSpeed(MAX_SPEED_R_MOTOR);
  stepperRadius.setAcceleration(5000.0);

  FastLED.clear(); // clear the LEDs
  FastLED.show();
  bleConfig.notifyStatus(String("[LEDINIT] pin=") + LED_DATA_PIN + " count=" + NUM_LEDS + " maxBrt=" + MAX_BRIGHTNESS);

  PatternScriptUnits units;
  units.stepsPerCm = STEPS_PER_MM * 10.0f;
  units.stepsPerDeg = (float)STEPS_PER_DEG;
  units.maxRadiusCm = (float)MAX_R_STEPS / units.stepsPerCm;
  configurePatternScriptUnits(units);
  clearActiveScript("");
  String presetInitErr;
  if (compileSandScriptPreset(0, presetInitErr))
  {
    bleConfig.notifyStatus(String("[SCRIPT] PRESET ") + kSandScriptPresets[0].name + " ready");
  }
  else if (presetInitErr.length())
  {
    bleConfig.notifyStatus(String("[SCRIPT] PRESET_ERR ") + presetInitErr);
  }

  // (Removed startup LED self-test)

  homeRadius(); // crash home the radial axis. This is a blocking function.

  bleConfig.begin(&bleListener);
  bleConfig.notifyStatus("Ready");
}

// (Removed LED self-test and LED scan utilities)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MAIN LOOP (runs endlessly).
This manages the state machine, tracks the position of the gantry, and acquires the target positions for
the gantry from the selected pattern functions or from manual mode.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  processPendingSandScript();
  bleConfig.loop(); // service BLE events if needed
  // (Removed LED direct, self-test, and scan debug handling)

  // Check to see if the button has been pressed. This has to be called as often as possible to catch button presses.
  button.tick();

  // Always read joystick once per outer loop and emit telemetry (throttled inside helper)
  joystickValues = readJoystick();

  if (runPattern)
  {
#pragma region Running
    // make sure the motors are enabled, since we want to move them
    if (!motorsEnabled)
    {
      stepperAngle.enableOutputs(); // enable the motor
      stepperRadius.enableOutputs();
      motorsEnabled = true; // set the state machine flag
    }

    // First we'll check the state machine flags to see if we're in manual drawing mode.
    if (!autoMode)
    { // Not in autoMode, which means we're in manual drawing mode
#pragma region ManualMode

      // Use the LEDs to indicate that we are in manual drawing mode
      display.setBrightness(MAX_BRIGHTNESS);
      display.indicatePattern(255); // pattern 255 is used to indicate manual drawing mode on the LEDs

      // (joystick already read at top of loop; telemetry already sent)
      // first check if an angular change is requested by joystick input
      int angCmd = joystickValues.angular;
      int radCmd = joystickValues.radial;
      if (abs(angCmd) < JOYSTICK_MANUAL_MOVE_THRESHOLD)
        angCmd = 0;
      if (abs(radCmd) < JOYSTICK_MANUAL_MOVE_THRESHOLD)
        radCmd = 0;

      if (angCmd < 0)
      {
        targetPositions.angular = currentPositions.angular - (STEPS_PER_MOTOR_REV / 100); // add steps to the target position
      }
      else if (angCmd > 0)
      {
        targetPositions.angular = currentPositions.angular + (STEPS_PER_MOTOR_REV / 100);
      }
      else
      {
        targetPositions.angular = currentPositions.angular; // otherwise maintain current position
      }
      // next check if a radial change is requested by joystick input
      if (radCmd < 0)
      {
        targetPositions.radial = currentPositions.radial - (MAX_R_STEPS / 100);
      }
      else if (radCmd > 0)
      {
        targetPositions.radial = currentPositions.radial + (MAX_R_STEPS / 100);
      }
      else
      {
        targetPositions.radial = currentPositions.radial;
      }

      // finally, take the steps necessary to move both axes to the target position in a coordinated manner and update the current position.
      currentPositions = orchestrateMotion(currentPositions, targetPositions);

#pragma endregion ManualMode

      // In this case, the state machine flags indicate that we're in automatic pattern mode, not manual mode.
    }
    else
    { // automatic pattern mode
#pragma region AutomaticMode
      // update the LED pattern display
      display.setBrightness(MAX_BRIGHTNESS);
      display.indicatePattern(currentPattern);

      // check to see if the pattern has been switched
      if (currentPattern != lastPattern)
      {
        patternSwitched = true;       // set the flag to indicate that the pattern has been changed
        lastPattern = currentPattern; // now we can say that the last patten is the current pattern so that this if block will be false until pattern is changed again
      }

      // Call the function that will generate the pattern.
      // This automatically calls the appropriate function from the patterns[] array.
      // Pass in the currentPositions as an argument, and the pattern function returns the targetPositions.
      // Note that the target positions are absolute coordinates: e.g., a pattern might say
      // to move to (radius, angle) = (1000 steps, 45 degrees (converted to steps)).
      // There is only one position on the sand tray that corresponds to those coordinates.
      targetPositions = patterns[currentPattern - 1](currentPositions, patternSwitched); // subtracing 1 here because I count patterns from 1, but the array that stores them is 0-indexed.
      bool debugRestart = patternSwitched;
      Positions debugTarget = targetPositions;

      patternSwitched = false; // after we've called the pattern function above, we can reset this flag to false.

      // finally, take the steps necessary to move both axes to the target position in a coordinated manner and update the current position.
      currentPositions = orchestrateMotion(currentPositions, targetPositions);

      if (g_debugStepStreaming)
      {
        emitPatternStepTelemetry(debugTarget, currentPositions, debugRestart);
      }

#pragma endregion AutomaticMode
    }
#pragma endregion Running
  }
  else
  { // In this case, runPattern is false, which means this is pattern selection mode
#pragma region SelectionMode

    // if the motors are enabled, disable them to save power while they don't need to run
    if (motorsEnabled)
    {
      stepperAngle.disableOutputs();
      stepperRadius.disableOutputs();
      motorsEnabled = false;
    }

    // joystick already read at top of loop

    if (!autoMode)
    {                                                      // This means we're not in automatic mode, so we are in manual drawing mode.
      display.indicatePattern(255);                        // The value 255 is used to represent manual mode on the LEDs.
      display.fadePixels(LED_FADE_PERIOD, MAX_BRIGHTNESS); // update the brightness of the LEDs to fade them in and out over time

      if (lastJoystickUpdate >= 200 && (joystickValues.angular >= 90 || joystickValues.angular <= -90))
      {                         // the joystick is pushed all the way to the right or left
        setAutoModeLocal(true); // switch to automatic mode so that we can do pattern selection
        lastJoystickUpdate = 0; // reset the joystick update timer
      }
    }
    else
    { // We're in automatic mode, which means it's time to select a pattern.
      display.indicatePattern(currentPattern);
      display.fadePixels(LED_FADE_PERIOD, MAX_BRIGHTNESS);
      // telemetry already emitted at top of loop

      if (lastJoystickUpdate >= 200 && joystickValues.radial >= 90)
      {                                      // if it's been 200ms since last joystick update and joystick is pushed all the way up
        setPatternLocal(currentPattern + 1); // increment pattern number via BLE sync helper
        lastJoystickUpdate = 0;              // reset the timer that tracks the last time the joystick was updated
      }
      else if (lastJoystickUpdate >= 200 && joystickValues.radial <= -90)
      { // if it's been 200ms since last update and joystick is pushed all the way down
        setPatternLocal(currentPattern - 1);
        lastJoystickUpdate = 0;
      }
      else if (lastJoystickUpdate >= 200 && (joystickValues.angular >= 90 || joystickValues.angular <= -90))
      {                          // if the joystick was pushed all the way to the left
        setAutoModeLocal(false); // switch to manual mode
        lastJoystickUpdate = 0;
      }
    }
#pragma endregion SelectionMode
  }

  // Periodic telemetry heartbeat
  heartbeatTelemetry();

  // Occasional aggregate snapshot when key state changed
  static uint8_t lastPatSent = 0;
  static bool lastAuto = !autoMode;
  static bool lastRun = !runPattern;
  static uint8_t lastBrt = 0;
  uint8_t brt = FastLED.getBrightness();
  if (lastPatSent != currentPattern || lastAuto != autoMode || lastRun != runPattern || abs((int)brt - (int)lastBrt) >= 16)
  {
    sendStateTelemetry();
    lastPatSent = currentPattern;
    lastAuto = autoMode;
    lastRun = runPattern;
    lastBrt = brt;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Miscellaneous functions. Currently this region only includes the function for reading the values of the joystick.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region MiscFunctions

/**
 * @brief Reads the analog values from the joystick potentiometers and returns them as a Positions struct.
 *
 * This function reads the analog input values from the joystick's potentiometers on the specified pins,
 * maps the values to a range of -100 to 100 for the angular axis and 100 to -100 for the radial axis,
 * and applies a deadband to eliminate small fluctuations around the center.
 *
 * @return Positions - a struct containing the mapped and processed values of the angular and radial joystick positions.
 *
 * The deadband ensures that values near the center of the joystick are treated as zero to account for
 * measurement noise and prevent unintended small movements.
 */

// Calibration constants (from user logs)
const int A_LOW = 2213;   // assumed low activation threshold (approx where motion begins toward RIGHT)
const int A_CENTER = 3151; // neutral angular
const int A_HIGH = 4095;   // full scale (LEFT)

const int R_LOW = 2213;    // assumed low activation threshold (approx where motion begins toward UP)
const int R_CENTER = 3154; // neutral radial
const int R_HIGH = 4095;   // full scale (DOWN)

// Deadzone half-width around center (tunable)
const int DEADZONE_A = 50; // raw counts around center for angular
const int DEADZONE_R = 40; // raw counts around center for radial
Positions readJoystick(void) {
  Positions values;
  values.angular = map(analogRead(JOYSTICK_A_PIN), A_LOW, A_HIGH, -100, 100);
  values.radial = map(analogRead(JOYSTICK_R_PIN), R_LOW, R_HIGH, 100, -100);

  if (values.angular <= DEADZONE_A && values.angular >= -DEADZONE_A) values.angular = 0;   //apply a deadband to account for measurement error near center.
  if (values.radial <= DEADZONE_R && values.radial >= -DEADZONE_R) values.radial = 0;
  return values;
}

#pragma endregion MiscFunctions

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This region of code contains all the functions related to calculating and performing the motion of the gantry.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Motion

/**
 * @brief Calculates the effective radial change, accounting for the motion of the angular axis.
 *
 * The radial axis movement is influenced by the angular axis movement, so this function computes the
 * actual change in the radial axis by considering the steps taken by both the angular and radial motors.
 *
 * @param angularMoveInSteps The number of steps the angular motor has moved.
 * @param radialMoveInSteps The number of steps the radial motor has moved.
 *
 * @return int The effective radial change in steps, with the angular axis movement accounted for.
 *         A positive value indicates a decrease in radius, while a negative value indicates an increase in radius.
 */
int calcRadialChange(int angularMoveInSteps, int radialMoveInSteps)
{
  int actualChangeR = angularMoveInSteps - radialMoveInSteps;

  // should return the number of steps R axis has moved, with A axis motion accounted for.
  // if actualChangeR is positive, radius is decreasing.
  // if actualChangeR is negative, radius is increasing.
  return actualChangeR;
}

/**
 * @brief Moves both the angular and radial motors to their target positions simultaneously.
 *
 * This function performs relative movements of the motors by taking in the number of steps
 * required for each motor to reach its target. One motor will move at maximum speed, while the
 * speed of the other motor is scaled to ensure both motors reach their target positions at the
 * same time. Note that this is a blocking function, meaning no other code will execute while
 * the motors are moving.
 *
 * @param angularSteps The number of steps the angular motor needs to move to reach its target.
 * @param radialSteps The number of steps the radial motor needs to move to reach its target.
 *
 * The function adjusts the speed of the motors proportionally based on the distance each motor
 * needs to travel, ensuring they complete their movements simultaneously. It also reduces the
 * maximum speed of the angular motor based on the current radial position to avoid excessive
 * speed at the outer edges.
 *
 * The function checks the state of the run/stop button during execution to allow for immediate
 * termination of the movement if needed.
 */
void moveToPosition(long angularSteps, long radialSteps)
{
  long absStepsA = abs(angularSteps), absStepsR = abs(radialSteps); // absolute values used to compare total distance each motor travels

  // Fetch the CURRENT configured max speeds (already include BLE multiplier effects)
  const float configuredMaxA = stepperAngle.maxSpeed();
  const float configuredMaxR = stepperRadius.maxSpeed();

  // Preserve original floor ratio: original code reduced 550 -> 150 at outer radius.
  // Floor fraction = 150 / MAX_SPEED_A_MOTOR.
  const float floorFraction = 150.0f / (float)MAX_SPEED_A_MOTOR; // ~0.2727
  const float floorA = configuredMaxA * floorFraction;           // scaled floor so multiplier affects both ends

  // Linear slope so that at radial = 0 => configuredMaxA, at radial = MAX_R_STEPS => floorA
  const float slopeA = (configuredMaxA - floorA) / (float)MAX_R_STEPS; // steps/sec per radial step
  float maxSpeedA = configuredMaxA - slopeA * currentPositions.radial;
  if (maxSpeedA < floorA)
    maxSpeedA = floorA; // clamp (safety)

  float maxSpeedR = configuredMaxR; // radial not currently derated by position

  float speedA = maxSpeedA;
  float speedR = maxSpeedR;
  float moveTime = 0.0f;

  // Determine which axis has the longer move and proportionally scale the other so both finish together.
  if ((absStepsA > absStepsR) && (absStepsA != 0))
  {
    moveTime = (float)absStepsA / maxSpeedA;      // seconds at A max
    speedR = (absStepsR == 0) ? 0.0f : (float)absStepsR / moveTime; // sync R
    if (speedR > maxSpeedR)
      speedR = maxSpeedR; // never exceed configured cap
  }
  else if ((absStepsR > absStepsA) && (absStepsR != 0))
  {
    moveTime = (float)absStepsR / maxSpeedR;      // seconds at R max
    speedA = (absStepsA == 0) ? 0.0f : (float)absStepsA / moveTime; // sync A
    if (speedA > maxSpeedA)
      speedA = maxSpeedA; // enforce derated angular cap
  }
  // If equal distances, both remain at their (possibly derated) max speeds.

  // Configure the moves (relative distances). Using runSpeedToPosition so set constant speeds directly.
  stepperAngle.move(angularSteps);
  stepperAngle.setSpeed((angularSteps < 0) ? -speedA : speedA); // respect direction explicitly for clarity
  stepperRadius.move(radialSteps);
  stepperRadius.setSpeed((radialSteps < 0) ? -speedR : speedR);

  // execute steps at the correct speed as long as a motor still needs to travel, and as long as the run/stop
  // button has not been pressed. If the runPattern flag is false, this loop will immediately exit,
  // leaving steps unfinished in the targeted move. There is code in the main loop after the call to moveToPosition()
  // that deals with this.

  // this is a blocking section. The only thing that can happen here is moving the motors and updatting the button state.
  // Adding more functionality inside this loop risks losing synchronization of the motors.
  while (((stepperAngle.distanceToGo() != 0) || (stepperRadius.distanceToGo() != 0)) && runPattern)
  {
    stepperAngle.runSpeedToPosition(); // constant speed move, unless the target position is reached.
    stepperRadius.runSpeedToPosition();
    button.tick(); // This blocking loop can potentially last a long time, so we have to check the button state.
    // Poll joystick at 10Hz during blocking move to keep telemetry flowing in automatic runs
    static unsigned long lastJoyPoll = 0;
    unsigned long now = millis();
    if (now - lastJoyPoll >= 100)
    {
      lastJoyPoll = now;
      Positions tmp = readJoystick();
    }
  }
}

/**
 * @brief Performs crash homing on the radial axis at startup.
 *
 * This function moves the radial axis to its home position by driving the motor past the known range
 * to ensure a hard stop at the mechanical limit. It allows the homing process to be interrupted early
 * by a long press of the joystick button if the ball reaches the center of the sand garden.
 *
 * @details The function moves the radial axis at a high speed without acceleration to reduce torque
 * when it reaches the mechanical stop. During the homing sequence, the function updates the LED display
 * and checks for a long press of the joystick button to potentially terminate the homing process early.
 * After reaching the stop, the function retracts the motor slightly to create a soft stop, releases any
 * tension in the mechanism, and sets the current motor position as the origin (0,0).
 *
 * @note This function sets the current position of both the angular and radial motors to zero after homing.
 *
 * @return void
 */
void homeRadius()
{
  stepperRadius.move(1.1 * ACTUAL_LEN_R_STEPS); // Longer than actual length of axis to ensure that it fully crash homes.
  stepperRadius.setSpeed(600.0);                // move fast without accelerations so that the motor has less torque when it crashes.
  while (stepperRadius.distanceToGo() != 0 && !buttonLongPressed)
  {                                     // run the R axis toward 0 for the entire length of the axis. Crash homing.
    stepperRadius.runSpeedToPosition(); // non-blocking move function. has to be called in while loop.
    display.homingSequence(false);      // display the homing sequence pattern on the LEDs
    button.tick();                      // poll the button to see if it was long pressed
  }
  buttonLongPressed = false;
  stepperRadius.stop();

  delay(100); // brief delay.

  stepperRadius.move(-1 * (HOMING_BUFFER + RELAXATION_BUFFER)); // move away from 0 to create a soft stop. RELAXATION_BUFFER releases tension in bead chain/flexible structures
  stepperRadius.runToPosition();                                // blocking move.

  stepperRadius.setCurrentPosition(0); // set the current positions as 0 steps.
  stepperAngle.setCurrentPosition(0);  // The current locations of the motors will be the origins of motion.

  currentPositions.angular = 0; // set the global current position variables to 0.
  currentPositions.radial = 0;
  display.homingSequence(true); // now that homing is done, display the homing complete sequence on the LEDs
}

/**
 * @brief Calculates the shortest path to the target position on the angular axis.
 *
 * This function determines the shortest distance required to move from the current position to the
 * target position on a circular axis, considering both clockwise and counterclockwise directions.
 * It returns the shortest distance, taking into account a wraparound value for circular motion.
 *
 * @param current The current position on the angular axis, in steps.
 * @param target The desired target position on the angular axis, in steps.
 * @param wrapValue The wraparound point for the axis (e.g., the total number of steps per revolution).
 *
 * @return int The shortest distance, in steps, required to move to the target position.
 *         Positive values indicate clockwise movement, while negative values indicate counterclockwise movement.
 */
int findShortestPathToPosition(int current, int target, int wrapValue)
{
  int dist1 = modulus((target - current), wrapValue);
  int dist2 = -1 * modulus((current - target), wrapValue);
  if (abs(dist1) <= abs(dist2))
  {
    return dist1;
  }
  else
  {
    return dist2;
  }
}

/**
 * @brief Calculates the number of steps required for the radial axis motor to move, accounting for the angular axis motion.
 *
 * This function computes the necessary steps for the radial axis motor to move from the current position
 * to the target position. It compensates for the fact that the angular axis motion influences the radial
 * axis but not vice versa. The function adjusts the radial movement based on the planned angular axis movement.
 *
 * @param current The current position of the radial axis in steps.
 * @param target The desired target position of the radial axis in steps.
 * @param angularOffsetSteps The number of steps the angular axis motor will move in the next planned move.
 *
 * @return int The total number of steps the radial axis motor needs to move, adjusted for the angular axis offset.
 */
int calcRadialSteps(int current, int target, int angularOffsetSteps)
{
  return ((current - target) + angularOffsetSteps);
}

/**
 * @brief Manages the entire process of moving both the angular and radial motors to their target positions.
 *
 * This function is responsible for coordinating the motion of both motors, ensuring that the angular
 * values wrap correctly, that the radial target stays within the defined limits, and that the radial
 * movement compensates for any angular axis movement. It encapsulates the series of steps required to
 * calculate the necessary movements, execute them, and update the current positions.
 *
 * @param currentPositions The current position of both the angular and radial axes, represented as a Positions struct.
 * @param targetPositions The desired target position for both the angular and radial axes, represented as a Positions struct.
 *
 * @return Positions The updated current positions of both the angular and radial axes after the motion has been executed.
 *
 * This function wraps the angular target around the 360-degree/0-degree transition point and calculates the shortest path
 * to the target. It also ensures that the radial position stays within its limits, compensates for the mechanical
 * relationship between the axes, and updates the current position after the move. If the move is interrupted (e.g.,
 * by a long joystick press), the current position tracking adjusts accordingly.
 */
Positions orchestrateMotion(Positions currentPositions, Positions targetPositions)
{
  // First take care of making sure that the angular values wrap around correctly,
  targetPositions.angular = modulus(targetPositions.angular, STEPS_PER_A_AXIS_REV);                                              // wrap value around the 360 degree/0 degree transition if needed
  targetPositions.angular = findShortestPathToPosition(currentPositions.angular, targetPositions.angular, STEPS_PER_A_AXIS_REV); // Find the shortest path to the new position.

  // First make sure the radial position target won't exceed the limits of the radial axis:
  targetPositions.radial = constrain(targetPositions.radial, 0, MAX_R_STEPS);

  // Update the radial target position based on how much the angular position is going to move.
  // This compensates for the mechanical link between the two axes. This also converts the absolute radial coordinate
  // into a relative coordinate, which stores how many steps the radial motor has to spin.
  targetPositions.radial = calcRadialSteps(currentPositions.radial, targetPositions.radial, targetPositions.angular);

  // execute the moves. This is a blocking function: it doesn't return until the move is complete.
  // Also note that these positions are relative coordinates. The pattern function generates an
  // absolute position as the target to move to, and then the lines of code after that calculate
  // how far the motors have to move in steps to get there. moveToPosition() takes those motor
  // steps as its arguments. So this function just tells the motors how far they have to move.
  moveToPosition(targetPositions.angular, targetPositions.radial);

  // Update the current position.
  // moveToPosition can be exited before the move is complete by long pressing the joystick button, so we have
  // to make sure that our position tracking system accounts for that. We also have to use the target positions
  // to update the current position.
  targetPositions.angular -= stepperAngle.distanceToGo();
  targetPositions.radial -= stepperRadius.distanceToGo();
  currentPositions.angular += targetPositions.angular;
  currentPositions.angular = modulus(currentPositions.angular, STEPS_PER_A_AXIS_REV); // wrap the anglular position around if it needs it.
  currentPositions.radial += calcRadialChange(targetPositions.angular, targetPositions.radial);

  return currentPositions;
}

#pragma endregion Motion

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Geometry Generation.
Functions that handle generating points and shapes for drawing. Draw straight lines, create polygons, perform the basic geometric transformations
like rotation, translation, scaling, and (eventually) reflection.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region GeometryGeneration

/**
 * @brief Precomputes and returns points approximating a straight line between two positions in polar coordinates.
 *
 * This function precomputes an array of points that approximate a straight line by interpolating between two
 * end points specified in cartesian coordinates, then converts them to polar coordinates. It stores these points
 * in a static array and returns the next point on each function call, allowing efficient streaming of precomputed
 * line points. The line is divided into a specified number of segments (resolution), with a maximum of 100 points.
 *
 * @param point0 The starting point of the line, specified in radial and angular steps.
 * @param point1 The ending point of the line, specified in radial and angular steps.
 * @param current The current position of the gantry, used to calculate relative motion if needed.
 * @param resolution The number of segments to divide the line into, defaulting to 100 and capped at 100.
 * @param reset Bool - set true to force recalculation for a new line.
 *
 * @return Positions The next point along the precomputed line, with radial and angular values in steps.
 *
 * @note The function handles vertical lines by temporarily rotating the points 90 degrees to avoid calculation
 * issues, then rotates them back before returning. The line is broken into segments up to the maximum length of
 * the array, and lines close to the center of the field are handled with a higher resolution to maintain accuracy.
 *
 * @details The first call to this function precomputes all points along the line, and subsequent calls return
 * each point in sequence. The function resets for a new line after the last point is returned.
 */
Positions drawLine(Positions point0, Positions point1, Positions current, int resolution = 100, bool reset = false)
{
  // this is the nested array that will store the precomputed points. has to be static so values persist between function calls.
  // it will be of the form pointArray[100][2] = {{r0, theta0}, {r1, theta1}, ... {r99, theta99}}.
  // to access the theta value for point3 (4th point in array), you would call point3.angular = pointArray[3][1];

  // Future update: make this a single layer array of type Positions instead of type Int for simplicity.
  static int pointArray[100][2];

  static int numPoints = 0;                           // the number of points the line will be approximated with.
  static bool newLine = true;                         // used to track if the function is being called for a new line, or if it needs to provide points for an extant line
  static float x0 = 0, x1 = 0, y0 = 0, y1 = 0;        // end points of the line
  static float xtemp = 0, ytemp = 0, thetaTemp = 0.0; // temporary storage for calculations
  static float stepover = 0;                          // how far to move along x-axis for interpolating along line
  static float m = 0;                                 // the slope of the line (y = mx + b)
  static float denom = 0;                             // the denominator in the slope calculation (x1 - x0)
  static float b = 0;                                 // the y-intercept of the line (y = mx + b, so b = y - mx. Use point0 values for y and x
  static bool pointsRotated = false;                  // used to indicate if points have been rotated to deal with vertical lines and need to be rotated back on output.

  Positions p0 = point0, p1 = point1; // containers for the points (we may need to modify their values to deal with vertical lines)
  Positions outputPoint;              // the struct we'll use for passing the target positions out of the function
  static int outNum = 0;              // used for tracking which point to return on each call to this function

  if (newLine || reset)
  {                                            // if this is a new line, or the reset flag is set
    numPoints = constrain(resolution, 0, 100); // we can approximate the line with up to 100 points. recalculate this number for each new line.

    // check now to see if there will be a vertical line after the coordinate transformation from polar to rectangular coords
    int comparisonA = STEPS_PER_A_AXIS_REV - max(p0.angular, p1.angular); // units are in steps
    int comparisonB = min(p0.angular, p1.angular);

    // this next step checks to see if the line connecting these two points is within half a degree of vertical in the rectangular coordinate system.
    // From my early testing, if the lines are more than half a degree off of vertical, they render perfectly fine without special handling.
    // It's really just a vertical line that gets weird (e.g., a line connecting two points that are 45 and 315 degrees off the origin ray at the same radius).
    if ((comparisonA - comparisonB <= convertDegreesToSteps(0.5)) && (comparisonA - comparisonB >= convertDegreesToSteps(-0.5)))
    {
      pointsRotated = true; // we're going to rotate the points by 90 degrees to deal with the nearly vertical line, so set this flag.
      p0.angular += convertDegreesToSteps(90);
      p1.angular += convertDegreesToSteps(90);
    }

    // take in the points, convert them to radians for the angular unit. only need to do this one time for a new line.
    // also convert each point from polar to cartesian coordinates.
    x0 = p0.radial * cos(convertStepsToRadians(p0.angular)); // x = r*cos(theta)
    y0 = p0.radial * sin(convertStepsToRadians(p0.angular)); // y = r*sin(theta)
    x1 = p1.radial * cos(convertStepsToRadians(p1.angular)); // x = r*cos(theta)
    y1 = p1.radial * sin(convertStepsToRadians(p1.angular)); // y = r*sin(theta)

    denom = x1 - x0;

    // calculate the slope
    m = (y1 - y0) / denom;
    // calculate the y-intercept   y = mx + b, so b = y - mx. Use point0 values for y and x
    b = y0 - (m * x0);

    if (b < 100.0 && b > -100.0)
    { // if the line is within 100 steps of the origin
      // This takes care of lines that come really close to intercepting the origin. First, I'm using this range of values rather
      // than saying if (b == 0.0) because this is using floating point math, and equalities like that almost never evaluate to
      // true with floats. Lines that come really close to the origin require the gantry to flip around 180 degrees in the
      // angular axis once the ball is at the center of the field. The straight line algorithm already handles this well, but if
      // the line is broken into a small number of segments, that large rotation at the center winds up drawing a small arc
      // around the center. I dealt with this by just having the program maximize the number of segments the lines is broken
      // into for lines which come close to the center. You can adjust the values in the condition above to change what it means
      // for a line to be close to the center to fine tune how well straight lines are drawn.
      numPoints = 100;
    }
    // This line doesn't come really close to intercepting the origin, so we'll handle it differently.

    // divide one axis into the number of segments required by resolution, up to a maximum of the length of the array they'll be stored in.
    // defining all of these values as static means the value will persist between function calls, but also means I have to reset them
    // to initial values once the last point in the line is returned.
    stepover = (x1 - x0) / (float)numPoints; // should define how far to move along x axis for interpolation along line.

    for (int i = 0; i < numPoints; i++)
    {
      // start by generating absolute position values for the points along the line in terms of {r, theta}.
      // We are starting with absolute values because the end points of the line are specified in absolute coordinates.

      if (i == 0)
      {                                // if it's the first point in the line, put the point0 values into the list to ensure we start there
        pointArray[i][0] = p0.radial;  // these units are already in steps as absolute coordinates
        pointArray[i][1] = p0.angular; // units in steps, absolute coordinates. need to be changed to relative later.
      }
      else if (i == numPoints - 1)
      { // If it's the last point in the line, put point1 values into the list to make sure we end there.
        pointArray[i][0] = p1.radial;
        pointArray[i][1] = p1.angular;
      }
      else
      { // We're somewhere along the line that isn't the beginning or end, so we need to generate these values.
        // Calculate the next x value in the series. Use the values of i and stepover to figure out how many line segments to increment along from the starting point.
        // I'm using (i + 1) instead of i in the calculation because I'm handling the first and last points separately,
        // so by the time we get to this code, we need to move over by at least one increment of stepover, but i starts counting from 0.
        xtemp = x0 + (i + 1) * stepover;
        ytemp = m * xtemp + b; // y = mx + b gives next y value in the series.

        // calculate the angular position of the current point.
        // atan2f(y, x) is a special version of the arctan function that returns the angle based on y and x.
        thetaTemp = atan2f(ytemp, xtemp);

        // ata2f() has a range of (-pi, pi), and we'll need to shift that to be (0, 2pi)
        if (thetaTemp < 0)
          thetaTemp = 2.0 * PI + thetaTemp; // this is in radians, ranging from 0 to 2pi

        // now that we know the anglular position of the point in radians, we need to find the radial position in units of steps
        // using the Pythagorean theorem (square roots calculate more efficiently than trig functions on Arduino Nano).
        // Then store the r and theta points in the array.
        pointArray[i][0] = sqrt(xtemp * xtemp + ytemp * ytemp); // the radial value of the point. This is absolute coordinates from the origin. Units are steps.
        // store the angular position converted from radians to steps. This is still in absolute coordinates, not relative.
        pointArray[i][1] = convertRadiansToSteps(thetaTemp);
      }

      // finally, if we rotated the points to deal with a vertical line, rotate them back.
      if (pointsRotated)
      {
        pointArray[i][1] -= convertDegreesToSteps(90);
      }
    }

    // we need to set the newLine flag to false so that the next time this function is called,
    // we can output the points along the line rather than recalculating the points.
    newLine = false; // later in the program, we have to reset this to true once the last point in the line of code is returned.
    reset = false;
    outNum = 0; // need to reset this to 0 so we can start outputting the points, starting from the first one.
    pointsRotated = false;
  }

  // now we need to output the correct point in the array.
  if (outNum < numPoints)
  {
    outputPoint.radial = pointArray[outNum][0];  // put the r value into the struct
    outputPoint.angular = pointArray[outNum][1]; // put the theta value into the struct
    outNum++;                                    // increment to the next point in the array
  }

  // once the last point is ready for return, reset all the variables necessary to rerun all the calculations on the next call to this function.
  if (outNum >= numPoints)
  {
    newLine = true;
  }

  // finally, return the value of the point to be moved to!
  return outputPoint;
}

/**
 * @brief Generates the vertices of a regular n-sided polygon (n-gon) and stores them in an array of Positions.
 *
 * This function computes the vertices of a regular polygon (n-gon) with a specified number of sides, radius,
 * center point, and optional rotation. The vertices are generated in polar coordinates, with the first vertex
 * starting at angle 0 (or rotated by the specified degrees) and are then translated to be centered around the
 * specified center point. The generated points are stored in the provided pointArray.
 *
 * @param pointArray A pointer to the array of Positions to be filled with the vertices of the polygon.
 * @param numPoints The number of vertices (or sides) of the polygon.
 * @param centerPoint The center point of the polygon, specified as a Positions struct (radial and angular coordinates).
 * @param radius The radius of the polygon, which is the distance from the center to each vertex (in motor steps).
 * @param rotationDeg An optional rotation of the polygon in degrees, defaulting to 0.0. This rotates the polygon around its center.
 *
 * @return void
 *
 * @note The function first generates the vertices centered on the origin in polar coordinates, then translates
 * them to the specified center point by converting to rectangular coordinates, performing the translation, and
 * converting back to polar. The translatePoints() function is used to handle this translation process.
 *
 * @example
 * // Example of generating an octagon with a radius of 4000 steps centered on the origin:
 * int numberVertices = 8;
 * Positions vertices[numberVertices];
 * Position center = {0, 0};
 * nGonGenerator(vertices, numberVertices, center, 4000, 0.0);
 *
 * // Example of generating a circle with 360 points and a radius of 2000 steps, centered at {3000, 60 degrees}:
 * int numberVertices = 360;
 * Positions vertices[numberVertices];
 * Position center = {3000, convertDegreesToSteps(60)};
 * nGonGenerator(vertices, numberVertices, center, 2000, 0.0);
 */
void nGonGenerator(Positions *pointArray, int numPoints, Positions centerPoint, int radius, float rotationDeg)
{
  //*pointArry is the pointer to the array that will be built out.
  // numPoints is the length of that array (equal to number of desired vertices).
  // centerPoint is the center point of the polygon (supply as a Position struct)
  // radius is the distance from the center point to a vertex. Units are motor steps.
  // rotationDeg rotates the polygon in degrees. The first vertex will always be at angle = 0, unless you specify a rotation angle.

  // Start by generating vertices in polar coords, centered on origin.
  int angleStep = STEPS_PER_A_AXIS_REV / numPoints; // calculate how much to step the angle over for each point

  for (int i = 0; i < numPoints; i++)
  {
    // define each vertex.
    // What i have done below is the same as doing:
    // pointArray[i].radial = radius; pointArray[i].angular = i * angleStep + convertDegreesToSteps(rotationDeg);
    // This is called aggregate initialization.

    pointArray[i] = {radius, i * angleStep + (int)convertDegreesToSteps(rotationDeg)};
  }

  // Currently all the points in the array are centered on the origin. We need to shift the points to be centered on the
  // desired center point. You can do this in polar coordinates, but it's much simpler to convert to rectangular coordinates,
  // move all the points, and then convert back to polar.

  if (centerPoint.radial != 0)
  {                                                      // if the radial coordinate of the center point is not 0, we need to translate the polygon
    translatePoints(pointArray, numPoints, centerPoint); // This moves all points in the array to be centered on the correct point
  }
}

/**
 * @brief Translates an array of points along a given translation vector, shifting their position in polar coordinates.
 *
 * This function translates the points in the pointArray by converting both the points and the provided
 * translation vector from polar to rectangular coordinates, performing the translation, and then converting
 * the points back to polar coordinates. It is useful for shifting polygons or target positions by a specified
 * offset. For example, this function can be used to shift the center of a polygon generated by nGonGenerator().
 *
 * @param pointArray A pointer to an array of points (of type Positions) representing the points to be translated.
 * @param numPoints The number of points in the array.
 * @param translationVector The translation vector to shift the points by, specified as a Positions struct.
 *
 * @return void - the array is modified directly because it is passed into this function as a pointer.
 *
 * @note The translation is performed by first converting the points and translation vector to rectangular
 * coordinates (x, y), adding the corresponding components, and then converting the updated points back to
 * polar coordinates (r, θ). This ensures that the points are translated accurately in both radial and angular
 * dimensions. The function assumes the angular component of the translation vector is in steps, and the
 * radial component is in motor steps.
 *
 * @example
 * // Example usage to shift a polygon to a new center:
 * Positions vertices[8];
 * Positions translationVector = {3500, convertDegreesToSteps(45)};
 * nGonGenerator(vertices, 8, {0, 0}, 4000, 0.0);
 * translatePoints(vertices, 8, translationVector);
 */
void translatePoints(Positions *pointArray, int numPoints, Positions translationVector)
{
  if (translationVector.angular != 0 || translationVector.radial != 0)
  { // desired polygon is not centered on origin, so we need to shift the points.
    for (int i = 0; i < numPoints; i++)
    {
      float x = pointArray[i].radial * cos(convertStepsToRadians(pointArray[i].angular));
      float y = pointArray[i].radial * sin(convertStepsToRadians(pointArray[i].angular));

      // now figure out where the center point is in rectangular coordinates
      // NOTE: at some point I want to move this calculation out of the for loop for efficiency
      float centerX = translationVector.radial * cos(convertStepsToRadians(translationVector.angular));
      float centerY = translationVector.radial * sin(convertStepsToRadians(translationVector.angular));

      // now use centerX and centerY to translate each point.
      x += centerX; // this should shift the X coordinate appropriately
      y += centerY; // this should shift the Y coordinate appropriately

      // now convert back into polar coordinates

      // calculate the angular position of the current point. Units are in radians.
      // atan2f(y, x) is a special version of the arctan function that returns the angle based on y and x.
      float angleTemp = atan2f(y, x);

      // atan2f() has a range of (-pi, pi), and we'll need to shift that to be (0, 2pi)
      if (angleTemp < 0)
        angleTemp = 2.0 * PI + angleTemp; // this is in radians, ranging from 0 to 2pi

      // now that we know the anglular position of the point in radians, we need to find the radial position in units of steps
      // using the Pythagorean theorem (square roots calculate more efficiently than trig functions on Arduino Nano).
      // Then store the r and theta points in the array.
      pointArray[i].radial = round(sqrt(x * x + y * y)); // the radial value of the point. This is absolute coordinates from the origin. Units are steps.

      // store the angular position converted from radians to steps. This is still in absolute coordinates.
      pointArray[i].angular = convertRadiansToSteps(angleTemp);
    }
  }
}

/*
NOT IMPLEMENTED.

The idea of this function is to take in an array of points that represent a shape, like a
hexagon generated by nGonGenerator, and use it to scale it up or down in size. I don't
yet have a great idea of how to solve this problem, so I left it here as a suggestion for
the hacker. Try your hand at solving this problem!
*/
// Removed unused geometry stubs: scalePoints, rotatePoints, reflectPoints

#pragma endregion GeometryGeneration

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This region of code contains useful math functions for doing things like converting between units, doing modulus math that doesn't allow negative
numbers, and finding the distance between points in polar coordinates.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Math

/**
 * @brief Maps a float value from one range to another.
 *
 * This function works similarly to the standard map() function but allows for floating-point inputs
 * and outputs. It maps a float n from a specified input range (in_min to in_max) to a corresponding
 * output range (out_min to out_max).
 *
 * @param n The float value to map.
 * @param in_min The lower bound of the input range.
 * @param in_max The upper bound of the input range.
 * @param out_min The lower bound of the output range.
 * @param out_max The upper bound of the output range.
 *
 * @return float The mapped value in the output range.
 */
float fmap(float n, float in_min, float in_max, float out_min, float out_max)
{
  return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Converts an angular measurement in degrees to the corresponding number of steps for a stepper motor.
 *
 * This function converts a given angle in degrees to the number of motor steps required for the stepper motor
 * to rotate by that angle. The conversion is based on the number of steps per full revolution of the motor.
 *
 * @param degrees The angle in degrees to convert.
 *
 * @return long The number of steps required for the motor to move the specified angle.
 */
long convertDegreesToSteps(float degrees)
{
  return round(fmap(degrees, 0.0, 360.0, 0.0, 2.0 * STEPS_PER_MOTOR_REV));
}

/**
 * @brief Converts an angular measurement in radians to the corresponding number of steps for a stepper motor.
 *
 * @param rads The angle in radians to convert.
 * @return long The number of steps required for the motor to move the specified angle in radians.
 */
long convertRadiansToSteps(float rads)
{
  return round(fmap(rads, 0.0, 2.0 * PI, 0.0, 2.0 * STEPS_PER_MOTOR_REV));
}

/**
 * @brief Converts a number of steps to the corresponding angle in radians.
 *
 * @param steps The number of motor steps to convert.
 * @return float The equivalent angle in radians.
 */
float convertStepsToRadians(float steps)
{
  return fmap(steps, 0.0, 2.0 * STEPS_PER_MOTOR_REV, 0.0, 2.0 * PI);
}

/**
 * @brief Converts a number of steps to the corresponding angle in degrees.
 *
 * @param steps The number of motor steps to convert.
 * @return float The equivalent angle in degrees.
 */
float convertStepsToDegrees(int steps)
{
  return fmap(float(steps), 0.0, 2.0 * STEPS_PER_MOTOR_REV, 0.0, 360.0);
}

/**
 * @brief Converts a distance in millimeters to the corresponding number of steps for the radial axis.
 *
 * @param mm The distance in millimeters to convert.
 * @return int The equivalent number of steps.
 */
int convertMMToSteps(float mm)
{
  return round(mm * STEPS_PER_MM);
}

/**
 * @brief Converts a number of steps to the corresponding distance in millimeters for the radial axis.
 *
 * @param steps The number of motor steps to convert.
 * @return float The equivalent distance in millimeters.
 */
float convertStepsToMM(float steps)
{
  return steps * MM_PER_STEP;
}

/**
 * @brief Computes the modulus of two integers, ensuring the result is non-negative.
 *
 * This function is a replacement for the % operator that prevents negative results by wrapping
 * negative values around to the positive range. It is mainly used for handling angular values
 * when the gantry wraps from 360 degrees to 0 degrees.
 *
 * @param x The dividend.
 * @param y The divisor.
 *
 * @return int The modulus result, always non-negative.
 */
int modulus(int x, int y)
{
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

/**
 * @brief Calculates the distance between two points in polar coordinates using the law of cosines.
 *
 * This function computes the distance between two points specified in polar coordinates (radii and angles).
 * It uses the law of cosines to perform the calculation, assuming the angles are provided in degrees
 * and the radii in arbitrary units. The returned distance is in the same units as the radii.
 *
 * @param p1 The first point, represented as a Positions struct (with radial and angular values).
 * @param p2 The second point, represented as a Positions struct (with radial and angular values).
 *
 * @return int The calculated distance between the two points, rounded to the nearest integer.
 */
// Removed unused function: calculateDistanceBetweenPoints

#pragma endregion Math

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This region of code contains the different pattern generating functions.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Patterns

Positions pattern_SandScript(Positions current, bool restartPattern)
{
  if (!g_activeScript.loaded)
  {
    g_lastSandScriptEvalInputs = SandScriptEvalInputsSnapshot();
    return current;
  }

  if (restartPattern)
  {
    resetActiveScriptRuntime();
  }

  const uint32_t evalIntervalUs = g_sandScriptIdleBackoffActive ? SANDSCRIPT_IDLE_EVAL_INTERVAL_US : SANDSCRIPT_MIN_STEP_INTERVAL_US;
  if (g_sandScriptStepTimer < evalIntervalUs)
  {
    if (g_sandScriptIdleBackoffActive)
    {
      delay(1);
    }
    else
    {
      yield();
    }
    return current;
  }
  g_sandScriptIdleBackoffActive = false;
  g_sandScriptStepTimer = 0;

  Positions next = evalPatternScript(g_activeScript.program, g_activeScript.runtime, current, restartPattern);

  if (g_activeScript.runtime.faulted) {
    String outputs;
    auto appendOutput = [&](uint8_t mask, const __FlashStringHelper *label) {
      if (g_activeScript.runtime.faultMask & mask) {
        if (outputs.length()) outputs += ',';
        outputs += label;
      }
    };
    appendOutput(PSG_MASK_NEXT_RADIUS, F("next_radius"));
    appendOutput(PSG_MASK_DELTA_RADIUS, F("delta_radius"));
    appendOutput(PSG_MASK_NEXT_ANGLE, F("next_angle"));
    appendOutput(PSG_MASK_DELTA_ANGLE, F("delta_angle"));
    if (!outputs.length()) {
      outputs = F("unknown");
    }
    String err = String("Runtime error: NaN/Inf in ") + outputs;
    g_activeScript.lastError = err;
    bleConfig.notifyStatus(String("[SCRIPT] RUNTIME_ERR ") + err);
    setRunLocal(false);
    return current;
  }

  SandScriptEvalInputsSnapshot snapshot;
  snapshot.radiusCm = convertStepsToMM(static_cast<float>(current.radial)) / 10.0f;
  snapshot.angleDeg = convertStepsToDegrees(current.angular);
  snapshot.rev = g_activeScript.runtime.unwrappedAngleDeg / 360.0f;
  snapshot.steps = (g_activeScript.runtime.stepCounter > 0) ? (g_activeScript.runtime.stepCounter - 1) : 0;
  uint32_t nowMs = millis();
  if (nowMs >= g_activeScript.runtime.startMillis)
  {
    snapshot.timeMs = nowMs - g_activeScript.runtime.startMillis;
  }
  else
  {
    snapshot.timeMs = 0;
  }
  snapshot.start = restartPattern;
  snapshot.valid = true;
  g_lastSandScriptEvalInputs = snapshot;

  next.radial = constrain(next.radial, 0, MAX_R_STEPS);
  next.angular = modulus(next.angular, STEPS_PER_A_AXIS_REV);

  const int radialDelta = abs(next.radial - current.radial);
  const int angularDelta = abs(findShortestPathToPosition(current.angular, next.angular, STEPS_PER_A_AXIS_REV));
  const bool noApparentMotion = (radialDelta <= SANDSCRIPT_IDLE_STEP_TOLERANCE_STEPS) && (angularDelta <= SANDSCRIPT_IDLE_STEP_TOLERANCE_STEPS);

  if (noApparentMotion)
  {
    if (g_sandScriptIdleSpinCount < SANDSCRIPT_IDLE_SPIN_THRESHOLD)
    {
      g_sandScriptIdleSpinCount++;
    }
    else
    {
      g_sandScriptIdleSpinCount = SANDSCRIPT_IDLE_SPIN_THRESHOLD; // clamp
      g_sandScriptIdleBackoffActive = true;
      if (!g_sandScriptIdleNotified)
      {
        bleConfig.notifyStatus("[SCRIPT] idle backoff engaged");
        g_sandScriptIdleNotified = true;
      }
    }
  }
  else
  {
    if (g_sandScriptIdleNotified)
    {
      bleConfig.notifyStatus("[SCRIPT] idle cleared");
      g_sandScriptIdleNotified = false;
    }
    g_sandScriptIdleSpinCount = 0;
  }
  return next;
}

/**
 * @brief Pattern: Simple Spiral. Generates the next target position for drawing a simple inward and outward spiral.
 *
 * This function calculates the next target position for a simple spiral pattern, starting from the current position.
 * The pattern progresses by incrementally adding small values to the current angular and radial positions. The spiral
 * moves inward more quickly than it moves outward due to the mechanical relationship between the radial and angular axes.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows for restarting the pattern (not used in this simple version). Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The pattern starts from the current position, so if the previous pattern leaves the ball in a specific position,
 * the spiral will continue from there. The pattern adjusts the radial and angular steps incrementally and reverses
 * direction when the radial boundaries are reached.
 */
Positions pattern_SimpleSpiral(Positions current, bool restartPattern)
{
  Positions target; // This is where we'll store the value of the next target position.

  const float angleDivisions = 100.0;                   // Going to divide a full revolution into 100ths. "const" because this never changes.
  const int radialDivisions = 10 * (int)angleDivisions; // Going to divide the radial axis into 1000ths. Try changing the 10 to a different number, like 20.

  // Calculate how many degrees we'll move over in the angular axis for the next step.
  const int angleStep = convertDegreesToSteps(360.0 / angleDivisions);

  // Calculate how far along we'll move the radial axis for the next step.
  // The "static" keyword means that this variable is defined once when the function is run for the first time.
  // This is different than "const" because this is a variable, not a constant, so we can still change the value.
  // If the following line were to omit the "static" keyword, this variable would be reset to its initial value
  // every time the function is called, meaning that we couldn't change it between positive and negative to
  // make the spiral grow inward or outward.
  static int radialStep = MAX_R_STEPS / radialDivisions;

  target.angular = current.angular + angleStep; // Set the angular position of the new point to move to (target position)
  target.radial = current.radial + radialStep;  // Set the radial target position as the current radial position plus the radialStep

  if (target.radial > MAX_R_STEPS || target.radial < 0)
  {                                  // Logic to see if the targeted radial position is out of bounds of the radial axis
    radialStep *= -1;                // If we're out of bounds, switch the sign of the radialStep (now we move in the opposite direction)
    target.radial += 2 * radialStep; // We were already out of bounds, so we undo that by moving 2 radialSteps in the new direction.
  }

  return target; // Return the target position so that the motion control functions can move to it.
}

/**
 * @brief Pattern: Cardioids. Generates the next target position for drawing repeating, slowly rotating cardioids.
 *
 * This function generates the next target position for a cardioid pattern, moving in relative coordinates by adding 43 degrees
 * to the current angular position and adjusting the radial position by 1/8th of the total radial axis. The pattern alternates
 * the direction of radial movement, creating a stepped approximation of a triangle wave along the radial axis.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern, setting the angular and radial positions to 0. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern works best after a reset, as it always operates in relative coordinates. If started after running another pattern,
 * the results may vary, since it builds upon the current position of the gantry.
 */
Positions pattern_Cardioids(Positions current, bool restartPattern)
{
  Positions target;
  const int radialStep = ((MAX_R_STEPS) / 8); // we're going to take huge steps radially (this defaults to 1/8th of the radial axis)
  static int direction = 1;                   // 1 means counterclockwise, -1 means clockwise
  static bool firstRun = true;

  if (firstRun || restartPattern)
  { // if it's the first time we're running the pattern, or if we start it from another pattern
    target.angular = 0;
    target.radial = 0;
    firstRun = false;
  }
  else
  {

    target.angular = current.angular + convertDegreesToSteps(43); // add 43 degrees to current position

    // this block of code moves the radial axis back and forth in increments that are 1/8th the length of the total radial axis.
    // Basically, this is a stepped approximation of a triangle wave.

    int nextRadial = current.radial + (direction * radialStep); // calculate potential next radial position

    if ((nextRadial <= MAX_R_STEPS) && (nextRadial >= 0))
    {                             // If the next radial position is in bounds of the radial axis soft limits
      target.radial = nextRadial; // Moves the radial axis positive direction by 1/8th the length of the axis
    }
    else
    {
      direction *= -1; // switch the radial step direction
      target.radial = current.radial + (direction * radialStep);
    }
  }

  return target;
}

/**
 * @brief Pattern: Wavy Spiral. Generates the next target position for drawing a wavy spiral pattern.
 *
 * This function creates a wavy spiral pattern, which is similar to the simple spiral pattern but with an additional sine wave
 * component added to the radial position. The result is a spiral with oscillating radial movement, creating a wavy effect.
 * The sine wave's amplitude and frequency can be adjusted to control the wave's characteristics.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern adds a sine wave to the radial position to create the wavy effect. You can modify the amplitude and frequency
 * of the wave to achieve different variations of the pattern. The radial movement is reversed when the limits of the radial axis are reached.
 */
Positions pattern_WavySpiral(Positions current, bool restartPattern)
{

  Positions target; // This is where we'll store the value of the next target position.

  float angleDivisions = 100.0;                   // Going to divide a full revolution into 100ths. "const" because this never changes.
  int radialDivisions = 10 * (int)angleDivisions; // Going to divide the radial axis into 1000ths. Try changing the 10 to a different number, like 20.

  // Add in values for the amplitude and frequency of the sine wave
  float amplitude = 200.0;
  int period = 8;

  // Calculate how many degrees we'll move over in the angular axis for the next step.
  const int angleStep = convertDegreesToSteps(360.0 / angleDivisions);

  static int radialStep = MAX_R_STEPS / radialDivisions;

  target.angular = current.angular + angleStep; // Set the angular position of the new point to move to (target position)
  target.radial = current.radial + radialStep;  // Set the radial target position as the current radial position plus the radialStep

  if (target.radial > MAX_R_STEPS || target.radial < 0)
  {                                  // Logic to see if the targeted radial position is out of bounds of the radial axis
    radialStep *= -1;                // If we're out of bounds, switch the sign of the radialStep (now we move in the opposite direction)
    target.radial += 2 * radialStep; // We were already out of bounds, so we undo that by moving 2 radialSteps in the new direction.
  }

  target.radial += (int)(amplitude * sin(period * convertStepsToRadians(target.angular)));

  return target; // Return the target position so that the motion control functions can move to it.
}

/**
 * @brief Pattern: Rotating Squares. Generates the next target position for drawing rotating squares, each rotated by 10 degrees.
 *
 * This function draws squares of the same size by connecting four points in sequence and rotating the square by 10 degrees
 * after completing each one. The function uses a switch-case statement to control the drawing process, ensuring each side
 * of the square is drawn in order. Once a square is complete, the vertices are rotated for the next iteration.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern relies on a static variable to track the current step in the drawing process and uses the drawLine function
 * to move between the vertices of the square. After each square is completed, the vertices are rotated by 10 degrees for the next square.
 */
Positions pattern_RotatingSquares(Positions current, bool restartPattern)
{
  Positions target;
  static int step = 0;
  static int segments = 20;        // Use  20 points to approximate a straight line
  static Positions p1, p2, p3, p4; // the four vertices of our square
  static bool firstRun = true;
  const int angleShift = convertDegreesToSteps(10); // how much we'll rotate the square
  if (firstRun || restartPattern)
  {
    p1.angular = 0;   // angular position of first point in absolute coordinates
    p1.radial = 7000; // radial position of first point in absolute coordiantes (units are steps)
    p2.angular = convertDegreesToSteps(90);
    p2.radial = 7000;
    p3.angular = convertDegreesToSteps(180);
    p3.radial = 7000;
    p4.angular = convertDegreesToSteps(270);
    p4.radial = 7000;
    firstRun = false;
  }

  switch (step)
  {
  case 0:                                                  // if step == 0
    target = drawLine(p1, p2, currentPositions, segments); // target positions are the result of calling drawLine between points p1 and p2
    if ((target.angular == p2.angular) && (target.radial == p2.radial))
    {         // If we've reached the end of the line
      step++; // Increment the value of step so we can move on to the next line
    }
    break; // have to include "break;" to avoid case fall through

  case 1: // if step == 1
    target = drawLine(p2, p3, currentPositions, segments);
    if ((target.angular == p3.angular) && (target.radial == p3.radial))
    {
      step++;
    }
    break;

  case 2:
    target = drawLine(p3, p4, currentPositions, segments);
    if ((target.angular == p4.angular) && (target.radial == p4.radial))
    {
      step++;
    }
    break;

  case 3:
    target = drawLine(p4, p1, currentPositions, segments);
    if ((target.angular == p1.angular) && (target.radial == p1.radial))
    {
      step++; // incrementing again would take us to case 4, but we don't have that, so default gets called next
    }
    break;

  default:
    // assuming that the step number was out of bounds, so reset it
    step = 0;                 // start the square over
    target = current;         // set the target position to the current position just as a default for the default option in the switch statement.
    p1.angular += angleShift; // rotate all points in the square by 10 degrees
    p2.angular += angleShift;
    p3.angular += angleShift;
    p4.angular += angleShift;
    break;
  }

  return target;
}

/**
 * @brief Pattern: Pentagon Spiral. Generates the next target position for drawing a growing and shrinking pentagon spiral.
 *
 * This function creates a pentagon using the nGonGenerator function to generate the vertices and then iterates through
 * the vertices, connecting them with straight lines. After completing a pentagon, the radius of each vertex is adjusted
 * by a radial step value (radialStepover). When the radius exceeds the maximum or falls below zero, the direction of
 * the radial change is reversed, creating a pattern of growing and shrinking pentagons.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern does not use a switch-case statement for sequencing but instead iterates over a list of precomputed points
 * (the vertices of the pentagon) and adjusts the radial distance of each point to create a spiral effect. The vertices are
 * recalculated when a complete pentagon is drawn.
 */

Positions pattern_PentagonSpiral(Positions current, bool restartPattern)
{
  Positions target;                      // Output position will be stored here
  static int start = 0;                  // Index to the starting point of the line in the array
  static int end = 1;                    // Index to the end point of the line in the array
  static bool firstRun = true;           // Flag for tracking if a new polygon needs to be generated
  const int vertices = 5;                // Change this to make a different polygon
  static Positions vertexList[vertices]; // construct an array to store the vertices of the polygon
  static int radialStepover = 500;       // Amount to change the radius of the polygon each cycle

  if (firstRun || restartPattern)
  {
    nGonGenerator(vertexList, vertices, {0, 0}, 1000, 0.0); // generate the vertices of the polygon
    firstRun = false;                                       // Use already generated points next time this function is called
  }
  target = drawLine(vertexList[start], vertexList[end], current, 100); // draw the line between the appropriate points

  if ((target.angular == vertexList[end].angular) && // If the line is complete, need to move on to the next line
      (target.radial == vertexList[end].radial))
  {
    start++; // increment start and end points of the line in the array
    end++;
    start = modulus(start, vertices); // wrap around to beginning of array if needed
    end = modulus(end, vertices);
    if (start == 0 && end == 1)
    { // If we're onto a new iteration of the polygon
      for (int i = 0; i < vertices; i++)
      { // Increase or decrease the radius of each point
        int newR = vertexList[i].radial + radialStepover;
        if (newR > MAX_R_STEPS || newR < 0)
        {                             // If the radius is getting out of bounds
          radialStepover *= -1;       // Switch direction of radial change
          newR += 2 * radialStepover; // move the other way
        }
        vertexList[i].radial = newR; // change the radius for each point
      }
    }
  }
  return target; // return the new target position
}

/**
 * @brief Pattern: Hex Vortex. Generates the next target position for drawing a series of growing, shrinking, and rotating hexagons.
 *
 * This function generates a hexagon vortex pattern, where hexagons grow and shrink over time while rotating.
 * When the outer edge of the radial axis is reached, the ball moves along the rim before shrinking back inward.
 * The ball also dwells at the center of the field. The pattern is controlled using a switch-case sequence that
 * moves between the six vertices of the hexagon.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The hexagon grows and shrinks by adjusting the radius incrementally (radialStepover) and rotates
 * by shifting the angular positions of each vertex. The pattern reverses direction when the radius exceeds
 * the maximum limit or falls below zero.
 */
Positions pattern_HexagonVortex(Positions current, bool restartPattern)
{
  Positions target;
  static int step = 0; // using switch case to track steps again
  static int segments = 100;
  static Positions p1, p2, p3, p4, p5, p6; // vertices of the hexagon
  static bool firstRun = true;
  const int angleShift = convertDegreesToSteps(5);
  static int radialStepover = 350; // how much we'll increase or decrease the size of the hexagon each iteration
  static int radius = 1000;        // starting radius

  if (firstRun || restartPattern)
  {
    p1.angular = 0;
    p1.radial = radius;
    p2.angular = convertDegreesToSteps(60);
    p2.radial = radius;
    p3.angular = convertDegreesToSteps(120);
    p3.radial = radius;
    p4.angular = convertDegreesToSteps(180);
    p4.radial = radius;
    p5.angular = convertDegreesToSteps(240);
    p5.radial = radius;
    p6.angular = convertDegreesToSteps(300);
    p6.radial = radius;
    firstRun = false;
  }

  // the step sequencer works just like the rotating square example, but with more steps
  switch (step)
  {
  case 0:
    target = drawLine(p1, p2, currentPositions, segments);
    if ((target.angular == p2.angular) && (target.radial == p2.radial))
    {
      step++;
    }
    break;

  case 1:
    target = drawLine(p2, p3, currentPositions, segments);
    if ((target.angular == p3.angular) && (target.radial == p3.radial))
    {
      step++;
    }
    break;

  case 2:
    target = drawLine(p3, p4, currentPositions, segments);
    if ((target.angular == p4.angular) && (target.radial == p4.radial))
    {
      step++;
    }
    break;

  case 3:
    target = drawLine(p4, p5, currentPositions, segments);
    if ((target.angular == p5.angular) && (target.radial == p5.radial))
    {
      step++;
    }
    break;

  case 4:
    target = drawLine(p5, p6, currentPositions, segments);
    if ((target.angular == p6.angular) && (target.radial == p6.radial))
    {
      step++;
    }
    break;

  case 5:
    target = drawLine(p6, p1, currentPositions, segments);
    if ((target.angular == p1.angular) && (target.radial == p1.radial))
    {
      step++;
    }
    break;

  case 6:
    // reset to the beginning
    step = 0;
    target = current; // set the target position to the current position just as a default for the default option in the switch statement.

    p1.angular += angleShift; // rotate all points
    p2.angular += angleShift;
    p3.angular += angleShift;
    p4.angular += angleShift;
    p5.angular += angleShift;
    p6.angular += angleShift;

    if ((radius + radialStepover >= MAX_R_STEPS + 2000) || (radius + radialStepover <= 0))
      radialStepover *= -1;   // If we're too far out of bounds, switch directions
    radius += radialStepover; // increase or decrease the radius for the points

    p1.radial = radius;
    p2.radial = radius;
    p3.radial = radius;
    p4.radial = radius;
    p5.radial = radius;
    p6.radial = radius;

    break;

  default:
    // assuming that the step number was out of bounds, so reset it
    step = 0;
    target = current; // set the target position to the current position just as a default for the default option in the switch statement.
    break;
  }

  return target;
}

/**
 * @brief Pattern: Pentagon Rainbow. Generates the next target position for drawing an off-center pentagon that rotates and moves.
 *
 * This function creates a pentagon pattern that is off-center, moving the center of the pentagon to a new position and
 * rotating it slightly with each iteration. The pentagon is generated using nGonGenerator and translated to the
 * appropriate location, while the center and orientation are adjusted progressively.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The center of the pentagon is translated and rotated slightly on each iteration, creating a "rainbow" effect
 * as the pentagon appears in different positions. The nGonGenerator and translatePoints functions are used to
 * generate and move the pentagon.
 */
Positions pattern_PentagonRainbow(Positions current, bool restartPattern)
{
  Positions target;
  // target = current;
  static int start = 0;
  static int end = 1;
  static bool firstRun = true;
  const int vertices = 5;
  static Positions pointList[vertices];
  static int radialStepover = 500;
  const int shiftDeg = 2;
  static int angleShift = convertDegreesToSteps(shiftDeg);
  static int shiftCounter = 1;

  if (firstRun || restartPattern)
  {
    nGonGenerator(pointList, vertices, {0, 0}, 3000, 0.0); // create the polygon
    translatePoints(pointList, vertices, {4000, 0});       // move the polygon to the appropriate spot
    firstRun = false;
  }

  target = drawLine(pointList[start], pointList[end], current, 100);

  if ((target.angular == pointList[end].angular) && (target.radial == pointList[end].radial))
  {
    start++;
    end++;
    start = modulus(start, vertices);
    end = modulus(end, vertices);
    nGonGenerator(pointList, vertices, {0, 0}, 3000, shiftCounter * shiftDeg); // build a new polygon that is rotated relative to the previous one
    translatePoints(pointList, vertices, {4000, shiftCounter * angleShift});   // move to the correct point
    shiftCounter++;
  }
  return target;
}

/**
 * @brief Pattern: Random Walk 1. Generates random target positions, moving via the shortest path to each point.
 *
 * This function creates a random walk pattern by generating random target positions in both the radial and angular axes.
 * The motion controller moves the gantry via the shortest path to each randomly generated point, resulting in random arcs.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next randomly generated target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern moves the gantry using the shortest path between points, leading to random arc-shaped movements.
 */
Positions pattern_RandomWalk1(Positions current, bool restartPattern)
{
  Positions target;

  // Generate a random radial position within the bounds of your system.
  int randomRadial = random(0, MAX_R_STEPS + 1); // +1 because the upper bound is exclusive

  // Generate a random angular position within a full circle in steps.
  int randomAngular = random(0, STEPS_PER_A_AXIS_REV);

  // Set the target position to the randomly generated values.
  target.radial = randomRadial;
  target.angular = randomAngular;

  return target;
}

/**
 * @brief Pattern: Random Walk 2. Generates random target positions and moves in straight lines to each point.
 *
 * This function creates a random walk pattern by generating random target positions in both the radial and angular axes.
 * Unlike Random Walk 1, this version moves the gantry in straight lines to each random point by connecting the current
 * position to the random target using the drawLine function.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The function generates new random points once the gantry reaches the current random target and continues the random walk.
 */
Positions pattern_RandomWalk2(Positions current, bool restartPattern)
{
  Positions target;
  static Positions randomPoint, lastPoint = current;
  static bool makeNewRandomPoint = true;

  if (makeNewRandomPoint)
  {
    // Generate a random radial position within the bounds of your system.
    randomPoint.radial = random(0, MAX_R_STEPS + 1); // +1 because the upper bound is exclusive

    // Generate a random angular position within a full circle in steps.
    randomPoint.angular = random(0, STEPS_PER_A_AXIS_REV);
    makeNewRandomPoint = false;
  }

  // Set the target position to the randomly generated values.
  target = drawLine(lastPoint, randomPoint, current, 100);

  if (target.angular == randomPoint.angular && target.radial == randomPoint.radial)
  {
    makeNewRandomPoint = true; // next time we'll generate a new random point
    lastPoint = randomPoint;   // save this as the previous point for the next iteration
  }

  return target;
}

/**
 * @brief Pattern: Accidental Butterfly. Generates the next target position for drawing a butterfly-like pattern with oscillating radial and angular movement.
 *
 * This function creates a butterfly-shaped pattern by modifying a simple spiral pattern with sine and cosine waves that adjust both the radial
 * and angular positions. The radial and angular positions are oscillated to create the butterfly pattern. I was actually trying to do something
 * entirely different and accidentally made this butterfly.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The pattern adds sine and cosine-based offsets to both the radial and angular positions to create oscillating movements, leading to the butterfly shape.
 * The amplitude and frequency of the sine and cosine waves can be adjusted for different effects.
 */
Positions pattern_AccidentalButterfly(Positions current, bool restartPattern)
{
  // This pattern starts out exactly the same as pattern_SimpleSpiral. The only difference is that after calculating the next position
  // in the spiral, it adds the sine of the current angular position to the radial axis to make a wavy line.

  Positions target; // This is where we'll store the value of the next target position.

  const float angleDivisions = 100.0;                   // Going to divide a full revolution into 100ths. "const" because this never changes.
  const int radialDivisions = 10 * (int)angleDivisions; // Going to divide the radial axis into 1000ths. Try changing the 10 to a different number, like 20.

  // Add in values for the amplitude and frequency of the sine wave
  const float amplitude = 200.0;
  const int frequency = 8;

  // Calculate how many degrees we'll move over in the angular axis for the next step.
  const int angleStep = convertDegreesToSteps(360.0 / angleDivisions);

  // Calculate how far along we'll move the radial axis for the next step.
  static int radialStep = MAX_R_STEPS / radialDivisions;

  target.angular = current.angular + angleStep; // Set the angular position of the new point to move to (target position)
  target.radial = current.radial + radialStep;  // Set the radial target position as the current radial position plus the radialStep

  if (target.radial > MAX_R_STEPS || target.radial < 0)
  {                                  // Logic to see if the targeted radial position is out of bounds of the radial axis
    radialStep *= -1;                // If we're out of bounds, switch the sign of the radialStep (now we move in the opposite direction)
    target.radial += 2 * radialStep; // We were already out of bounds, so we undo that by moving 2 radialSteps in the new direction.
  }

  // Add a new component to the radial position to make it oscillate in and out as a sine wave.
  int rOffset = (int)(200.0 * sin(8 * convertStepsToRadians(target.angular)));
  int aOffset = (int)(40.0 * cos(3 * convertStepsToRadians(target.angular)));
  target.radial += rOffset;

  // Now do the same for the angular axis so we get some back and forth:
  target.angular += aOffset;

  return target; // Return the target position so that the motion control functions can move to it.
}

#pragma endregion Patterns

/*
NOTES:
  -the max distance the rack can travel is about 102.386mm (calibrated for the 10 cm working span plus buffer).
  -that's spread over 28 teeth of movement of the pinion.
  -that's 3.141678 mm per tooth. love how close that is to pi.
  -the pinion is 16 teeth.
  -1 full revolution of the pinion should move the rack 50.267mm.
  -the pulley ratio is 2:1, so two revs of the motor cause one rev of the pinion
  -there are 2048 steps per rev in the motor (may need to update with exact number).
  Note that this is an approximation since the gearbox on the motor is not an integer ratio.
  -4096 steps will drive the pinion one full revolution (so 4096 steps per 50.267mm)
  -that makes 70.0 steps per mm of rack travel.
  -or the inverse of that if needed: 0.01429 mm per step
  -to fully move the rack across ~102.386mm, need roughly 7167 steps.
  -round down to 7000 steps total, losing about 2.4mm of total travel, and use about 1.2mm on each end
  as a soft buffer on the travel to prevent crashing.



POSSIBLE IMPROVEMENTS TO MAKE:
- Add backlash compensation. The motor gearboxes have backlash, as do the drive belts and pulleys. Every time the motor
switches direction, it has to spin a bit to take up all the slack before actually moving the gantry. It works fine as it is,
but for greater precision, take direction changes into account and add extra steps to each move to take up the slack before
executing the desired movement.

- Add functionality for the rest of the geometric transformations: I already added translation, so add rotation, scaling,
and reflection.

- Add proportional mixing to manual drawing mode. Right now it just works like arrow keys. Mixing would make the control
more subtle and precise.

- Normalize the speed of the ball. Right now the ball travels in a range of speeds, but it would be cool to make a sytem
that would ensure that the ball always moves at the same speed. Requires more advanced path planning than what I have here.
Note that I have a hack in place for this that makes the speed of the angular motor inversely proportional to the radius.
It actually works pretty well, but isn't the same as normalized speed.
*/