# Sand Garden (ESP32 Edition)

This repository contains the ESP32 firmware and browser tools that drive the CrunchLabs Sand Garden. Upload the sketch to an Arduino Nano ESP32, drop the board into the original Sand Garden electronics bay, and you’ll get the familiar automatic patterns plus a brand-new scriptable slot that lets you design paths using a tiny math-friendly language called **SandScript**.

## What you need
- CrunchLabs Sand Garden hardware (two steppers, joystick, LED bar) with the stock wiring harness.
- Arduino Nano ESP32 (a.k.a. "Nano ESP32 / nano_nora" board) and a USB-C cable.
- A computer with the Arduino IDE or Arduino CLI, plus the ESP32 board package installed.
- Chrome or Edge if you want to use the optional Web Bluetooth controller/preview.

## How the pieces work together
- **Firmware (`sand-garden.ino`)** homes the axes, drives the built-in patterns, handles joystick manual mode, and keeps the LED bar in sync. Pattern slot 11 is reserved for SandScript.
- **SandScript runtime (`PatternScript.*`)** compiles short math expressions into motor targets. The same script runs on-device and in the browser preview so you can check results before touching the hardware.
- **BLE service (`BLEConfigServer.*`)** exposes speed, mode, run/stop, pattern select, and a script upload channel. Any Web Bluetooth client (the included web app or your own tool) can talk to it.
- **Web client (`web-client.html`)** previews legacy patterns, lets you draft SandScript, uploads scripts over BLE, and streams live telemetry so you can compare the simulated path with the real machine.

## Quick start (firmware)
1. Install the Arduino IDE or install Arduino CLI and the **Arduino ESP32** board package. Select the board **Arduino Nano ESP32**.
2. Clone or download this repository and open `sand-garden.ino`.
3. Connect the board by USB-C, choose the correct serial port, and upload the sketch. The project also ships with a VS Code task named **Arduino: Compile Sand Garden** if you prefer tasks.
4. Power the Sand Garden. On boot it will home the radial axis, light the LED bar, and sit in pattern-select mode. Press the joystick button to start the currently selected pattern; long-press exits the homing cycle early.

## SandScript in a nutshell
SandScript is a compact math DSL that produces the next radius/angle for the drawing head. Every evaluation exposes these read-only inputs:

`radius` (cm), `angle` (deg), `start` (1 on first step), `rev` (continuous revolution count), `steps` (evaluation counter), `time` (ms since script start).

Assign any of these outputs to drive motion (values are always in cm/deg): `next_radius`, `next_angle`, `delta_radius`, `delta_angle`. You can create temporary variables just by naming them.

Supported functions: `sin`, `cos` (degree-based), `abs`, `clamp(value, min, max)`, `sign`. Operators: `+ - * / %` plus parentheses and unary `-`. Comments begin with `#`.

Example script:
```
# Ease between the center and the rim while spinning slowly
wave = 0.5 * (1 - cos(rev * 180))
target = clamp(wave * 10, 0, 10)
next_radius = radius + (target - radius) * 0.25
next_angle = angle + 12
```

## Preview and upload from the browser
1. Open `web-client.html` in Chrome or Edge (Web Bluetooth is required). No server is needed—double-clicking the file works.
2. The **Visualizer** area shows the built-in firmware patterns. Switch to **Pattern 11 – SandScript** to preview your own code.
3. The **Sandscript** panel lets you edit, compile, and simulate scripts. When you like the result, click **Send to Device**. The app will prompt you to connect over BLE and stream the script using the built-in chunk protocol.
4. Enable the **Device Debug Stream** toggle to compare live device telemetry with the simulated path. Differences are highlighted so you can spot missed steps or calibration drift.

## Tips & troubleshooting
- The joystick shipping harness feeds 5 V; the ESP32 analog pins prefer ~3.3 V. For long-term use add a simple voltage divider per axis so the readings stay within spec.
- If the ball refuses to move after a reboot, check that the homing cycle completed (the LED bar flashes green). You can restart homing by power-cycling or long-pressing the joystick during boot.
- Use the web client’s status pane to inspect BLE messages. Commands such as `BLEADV`, `BLEDROP`, or `SCRIPT_PRESET list` can be sent from the browser when you need to recover the link or reload a preset script.
- Pattern slot 11 always runs the most recently compiled SandScript (preset or uploaded). Switching to another pattern and back will restart your script from its first step.

## Where to go next
- Craft new SandScript patterns and share them—short snippets can create dramatic petals, breathing waves, or mirrored motifs.
- Extend the firmware tests to cover more edge cases (division by zero, deep clamp nesting, token limits) or hook the planned parity harness into CI.
- Customize the web client with a richer editor (Monaco, syntax themes) or add MIDI/gamepad controls via Web Bluetooth.

Have fun building your own kinetic sand patterns! The combination of firmware, BLE, and SandScript gives you a friendly starting point without diving into the low-level motion code.


