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

Supported functions: `sin`, `cos` (degree-based), `abs`, `clamp(value, min, max)`, `sign`, `pingpong(value, max)`. Operators: `+ - * / %` plus parentheses and unary `-`. Comments begin with `#`.

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

<!-- pingpong documented above within supported functions and examples -->

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

[bluetooth]: .docs/bluetooth-connection.png "Bluetooth pairing prompt"
[pattern-selection]: .docs/pattern-selection.png "Pattern selection list"
[pattern-visualization]: .docs/pattern-visualization.png "Pattern visualizer"
[sandscript-editor]: .docs/sandscript-built-in-scripts.png "Sandscript editor and built-in scripts"
[sandscript-debug]: .docs/sandscript-debugging-and-device-reporting.png "Sandscript debugging & device reporting"
[device-telemetry]: .docs/device-position-tracking-vs-local-step-simulation.png "Device position vs local simulation"
[sandscript-language]: .docs/sandscript-pattern-scripting-language.png "Sandscript pattern scripting language"


