# Sand Garden (ESP32 Edition)

This repository contains the ESP32 firmware and browser tools that drive the CrunchLabs Sand Garden. Upload the sketch to an Arduino Nano ESP32, drop the board into the original Sand Garden electronics bay, and you’ll get the familiar automatic patterns plus a brand-new scriptable slot that lets you design paths using a tiny math-friendly language called **SandScript**.

## What you need
- CrunchLabs Sand Garden hardware.
- Arduino Nano ESP32 (a.k.a. "Nano ESP32 / nano_nora" board).
- A computer with the Arduino IDE or Arduino CLI, plus the ESP32 board package installed.
- Chrome or Edge if you want to use the optional Web Bluetooth controller/preview.

## How the pieces work together
- **Firmware (`sand-garden.ino`)** homes the axes, drives the built-in patterns, handles joystick manual mode, and keeps the LED bar in sync. Pattern slot 11 is reserved for SandScript.
- **SandScript runtime (`PatternScript.*`)** compiles short math expressions into motor targets. The same script runs on-device and in the browser preview so you can check results before touching the hardware.
- **BLE service (`BLEConfigServer.*`)** exposes speed, mode, run/stop, pattern select, and a script upload channel. Any Web Bluetooth client (the included web app or your own tool) can talk to it.
- **Web client (`web-client.html`)** previews legacy patterns, lets you draft SandScript, uploads scripts over BLE, and streams live telemetry so you can compare the simulated path with the real machine.

## Quick start (firmware)

### Required Arduino libraries

Before uploading the sketch, install the following Arduino libraries (use the Library Manager in the Arduino IDE or install via Arduino CLI):

- AccelStepper — controls the stepper motors (AccelStepper)
- FastLED — LED bar driver (FastLED)
- OneButtonTiny — simple button handling/debouncing (OneButtonTiny)
- elapsedMillis — lightweight timer helpers (elapsedMillis)
- NimBLE (NimBLE-Arduino) — BLE stack used by the ESP32 core (NimBLE-Arduino). Note: the NimBLE implementation is normally provided by the ESP32 Arduino board package; install "NimBLE-Arduino" if your setup is missing it.

You can install all of them with the Arduino CLI (example):

arduino-cli lib install "AccelStepper" "FastLED" "OneButtonTiny" "elapsedMillis" "NimBLE-Arduino"

If you prefer the Arduino IDE: open Sketch -> Include Library -> Manage Libraries..., then search for and install each library above.

1. Install the Arduino IDE or install Arduino CLI and the **Arduino ESP32** board package. Select the board **Arduino Nano ESP32**.
2. Clone or download this repository and open `sand-garden.ino`.
3. Connect the board by USB-C, choose the correct serial port, and upload the sketch. The project also ships with a VS Code task named **Arduino: Compile Sand Garden** if you prefer tasks.
4. Power the Sand Garden. On boot it will home the radial axis, light the LED bar, and sit in pattern-select mode. Press the joystick button to start the currently selected pattern; long-press exits the homing cycle early.

## SandScript in a nutshell
SandScript is a compact math Domain Specific Language (DSL) that produces the next radius/angle for the drawing head. Every evaluation exposes these read-only inputs:

`radius` (cm), `angle` (deg), `start` (1 on first step), `rev` (continuous revolution count), `steps` (evaluation counter), `time` (ms since script start).

Assign any of these outputs to drive motion (values are always in cm/deg): `next_radius`, `next_angle`, `delta_radius`, `delta_angle`. You can also create local temporary variables just by naming them.

Supported functions: `sin`, `cos`, `tan` (degree-based), `abs`, `clamp(value, min, max)`, `sign`, `pingpong(value, max)`, `min(a, b)`, `max(a, b)`, `pow(a, b)`, `sqrt(x)`, `exp(x)`, `random()`, `floor(x)`, `ceil(x)`, `round(x)`. Operators: `+ - * / %` plus parentheses and unary `-`. Comments begin with `#`.
`random()` emits values in `[0,1)` and reseeds whenever `start` is 1. `floor`, `ceil`, and `round` operate on the final floating-point value.

Example script:
```
# Ease between the center and the rim while spinning slowly
wave = 0.5 * (1 - cos(rev * 180))
target = clamp(wave * 10, 0, 10)
next_radius = radius + (target - radius) * 0.25
next_angle = angle + 12
```

Additional examples (pingpong):
```
# oscillate radius between 0 and 6 cm based on step counter
next_radius = pingpong(steps * 0.2, 6)

# time-based bounce between 0 and 4
next_radius = pingpong(time * 0.005, 4)
```

![Sandscript reference][sandscript-reference]

## Preview and upload from the browser
1. Open `web-client.html` in Chrome or Edge (Web Bluetooth is required). No server is needed—double-clicking the file works.
   
   ![Bluetooth pairing prompt][bluetooth]
 
   *Browser will prompt to pair with the Sand Garden device when you click "Connect".*

2. The **Visualizer** area shows the built-in firmware patterns. Switch to **Pattern 11 – SandScript** to preview your own code.
   
   ![Pattern selection and list][pattern-selection]
  
   *Choose a built-in pattern or switch to Slot 11 to run SandScript.*
3. The **Sandscript** panel lets you edit, compile, and simulate scripts. When you like the result, click **Send to Device**.
   
   ![Sandscript editor and built-in scripts][sandscript-editor]

   *Edit live in the browser and load one of the bundled SandScript presets.*

   ![Sandscript pattern scripting language and send button][sandscript-language]

   *The editing textarea and the "Send to Device" / preset buttons are shown above.*
4. Enable the **Device Debug Stream** toggle to compare live device telemetry with the simulated path. Differences are highlighted so you can spot missed steps or calibration drift.

   ![Device telemetry vs simulation][device-telemetry]

   *Live telemetry (bottom pane) compares the device-reported step/radius/angle with the browser simulation.*

## Tips & troubleshooting
- The joystick is connected by default to 5V provided from USBc; the ESP32 analog pins prefer ~3.3V. I am scaling this roughly in the sfotware, but it may be dangerous for microprocessor. For long-term use add a simple voltage divider per axis so the readings stay within spec.
- Use the web client’s status pane to inspect BLE messages.
- Pattern slot 11 always runs the most recently compiled SandScript (preset or uploaded). Switching to another pattern and back will restart your script from its first step.

 ![Visualizer showing a pattern][pattern-visualization]

 *The Visualizer renders the simulated path so you can preview motion before sending it to the device.*

## Where to go next
- Craft new SandScript patterns and share them—short snippets can create dramatic petals, breathing waves, or mirrored motifs.
- Share sandscript patterns on discord and add them with pull request to this repo! Let's build megapattern library!
- Customize the web client with a richer editor (Monaco, syntax themes) or add MIDI/gamepad controls via Web Bluetooth.

 ### More about SandScript

 SandScript is designed to be tiny and testable in the browser. The editor exposes both the compiled script and a small debug pane that reports the last inputs/outputs and a live stream of device state when connected.

 ![Sandscript debugging and device reporting][sandscript-debug]

 *Debugging pane shows last inputs/outputs and the live device step/radius/angle feed.*

Have fun building your own kinetic sand patterns! The combination of firmware, BLE, and SandScript gives you a friendly starting point without diving into the low-level motion code.

## Embedding SandScript into the firmware (no BLE)

If your board can't use BLE (or you prefer not to use the web client), you can embed SandScript patterns directly in the firmware and run them from the device using the joystick and button. The firmware already supports a small built-in preset table and a dedicated "SandScript" pattern slot — here is how to work with them.

### Where the built-in scripts live

- Open `sand-garden.ino` and look for the `kSandScriptPresets` table. Each entry is a simple { name, source } pair. Example entry:

```
{
   "BloomSpiral",
   "# Sand garden bloom spiral\n"
   "next_radius = clamp(radius + 0.18 * sign(sin(angle + rev * 45)), 0.6, 8.3)\n"
   "next_angle = angle + 18 + 10 * sin(rev * 60)\n"
},
```

Add, remove or edit entries in this array to include any scripts you want compiled into the firmware. Presets are stored as C string literals, so escape newlines and keep the total script length under the compile-time limit (see `PSG_MAX_SCRIPT_CHARS` in `PatternScript.h`).

### Default script on boot

The sketch compiles the first preset at startup by calling `compileSandScriptPreset(0, ...)`. To change which embedded preset is activated on boot, change that index (presets are zero-based), and optionally select the SandScript pattern slot so it runs immediately:

Example (edit in `setup()` near the existing preset init):

```
String presetInitErr;
if (compileSandScriptPreset(1, presetInitErr)) { // load preset #2 from the array (zero-based)
   // select the SandScript slot so the device runs it right away
   currentPattern = SCRIPT_PATTERN_INDEX; // pick the dynamic SandScript slot
   patternSwitched = true;                 // force a restart so the script starts from step 0
}
else {
   // fallback/logging
}
```

This approach avoids depending on BLE during boot — the script is compiled and the controller selects the script slot locally so you can start it with the joystick/button on the device.

### Selecting and running embedded presets on the device

- Pattern selection is available in the on-device selection UI (use the joystick). Push the radial axis up/down to increment/decrement the selected pattern. The LEDs show the current pattern number. When you reach the last pattern number, that slot is the SandScript slot (the sketch maps `SCRIPT_PATTERN_INDEX` to the SandScript runner).
- Press the joystick button (single click) to start/stop the currently selected pattern. Long-press is used to abort homing early during boot.

If you compiled a built-in preset at boot (see previous section), set `currentPattern = SCRIPT_PATTERN_INDEX` and press the joystick button to start it. If you want to switch between multiple embedded presets without BLE, you can either:

- Edit the firmware and change which preset is compiled on boot (rebuild/upload).
- Or add a simple local control hook in the sketch to cycle presets using a button event (example below). The example avoids BLE and prints status to `Serial` so it works on boards without BLE enabled:

Example: compile-and-run preset on double-click (add to `setup()`):

```
// Example: double-click cycles to preset index 1
button.attachDoubleClick([](){
   String err;
   if (compileSandScriptPreset(1, err)) { // preset index (zero-based)
      currentPattern = SCRIPT_PATTERN_INDEX; // select sandscript slot
      patternSwitched = true;
      Serial.println("[SCRIPT] PRESET loaded: index=1");
   } else {
      Serial.println("[SCRIPT] PRESET_ERR: " + err);
   }
});
```

You can adapt the handler to cycle through preset indices, or map other button events. Keep the preset indices zero-based and ensure you don't exceed `SANDSCRIPT_PRESET_COUNT`.

### Notes & limits

- Presets compiled into the firmware are subject to the same compile/runtime limits as uploaded scripts: see `PatternScript.h` (`PSG_MAX_SCRIPT_CHARS`, `PSG_MAX_TOKENS`, etc.).
- The SandScript slot is the last pattern in the array of built-in patterns. In the code it is exposed as `SCRIPT_PATTERN_INDEX` (1-based). Use that constant when you want the device to run the active script from firmware.
- If you add many or long scripts you may increase flash usage; keep preset count and script size modest on smaller ESP32 flash sizes.


[bluetooth]: .docs/bluetooth-connection.png "Bluetooth pairing prompt"
[pattern-selection]: .docs/pattern-selection.png "Pattern selection list"
[pattern-visualization]: .docs/pattern-visualization.png "Pattern visualizer"
[sandscript-editor]: .docs/sandscript-built-in-scripts.png "Sandscript editor and built-in scripts"
[sandscript-debug]: .docs/sandscript-debugging-and-device-reporting.png "Sandscript debugging & device reporting"
[device-telemetry]: .docs/device-position-tracking-vs-local-step-simulation.png "Device position vs local simulation"
[sandscript-language]: .docs/sandscript-pattern-scripting-language.png "Sandscript pattern scripting language"
[sandscript-reference]: .docs/sandscript-reference.png "Sandscript language reference"


