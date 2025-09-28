
## Changes since original

- Added an on-device SandScript compiler/VM with BLE upload pipeline, built-in presets, and status/error reporting. The last pattern slot now runs the most recently compiled script (preset or uploaded via BLE).
- Output pin number for A0 is not correct for some reason on ESP32. I had to specify `GPIO_NUM_1` manually for that pin.
- Joystick analog reads did not work as expected on ESP32 — the resting state was around 75% of the max value. The original Arduino Nano is 5V, but the ESP32 uses 3.3V logic. Since the joystick gets its VCC from USB-C (typically 5V), this can **overdrive the ESP32 analog pins**. Technically, you should use a voltage divider to safely drop the joystick output to 3.3V for the ESP32. For quick testing, I applied a naive offset in code, but **for long-term reliability, add a voltage divider on each analog line**.
 - Added explicit BLE server callbacks (`ServerCallbacks`) to restart advertising after a disconnect. Some host stacks (or rapid connect/disconnect cycles) caused advertising to stop after the first client session; now `onDisconnect` always calls `NimBLEDevice::getAdvertising()->start()` and emits a status line like `[BLE] DISCONNECT reason=0`.
 - Status characteristic now reports basic BLE lifecycle events: connect, disconnect (reason code), and negotiated MTU.

## Working

### Compilation
```bash
arduino-cli compile --fqbn arduino:esp32:nano_nora --warnings default --build-property "compiler.cpp.extra_flags=-fpermissive" --export-binaries .
```

### Upload
```bash
arduino-cli upload -p <PORT> --fqbn arduino:esp32:nano_nora .
```

### Compile & upload

```bash
arduino-cli compile --fqbn arduino:esp32:nano_nora --warnings default --build-property "compiler.cpp.extra_flags=-fpermissive" --export-binaries . &&  arduino-cli upload --fqbn arduino:esp32:nano_nora .
```

### BLE Configuration (Sand Garden)
A NimBLE GATT server is now exposed with device name `Sand Garden` providing a configuration service:

Service UUID:
- `9b6c7e10-3b2c-4d8c-9d7c-5e2a6d1f8b01`

Characteristics:
- Speed Multiplier (RW, Notify) UUID `9b6c7e11-3b2c-4d8c-9d7c-5e2a6d1f8b01`
  - ASCII float (e.g. `1.0`, `0.75`, `2.000`) or raw 4-byte IEEE754 float write.
  - Adjusts both axis max speeds proportionally (clamped 0.1x .. 3.0x).
- Current Pattern (RW, Notify) UUID `9b6c7e12-3b2c-4d8c-9d7c-5e2a6d1f8b01`
  - ASCII integer (1..N) or raw 4-byte int write.
  - Changes the active pattern and forces restart logic.
- Status (Read, Notify) UUID `9b6c7e13-3b2c-4d8c-9d7c-5e2a6d1f8b01`
  - Human-readable status messages (e.g. `Boot`, `Ready`).
  - Now also includes BLE lifecycle logs like `[BLE] CONNECT`, `[BLE] DISCONNECT reason=<code>`, `[BLE] MTU=<n>`.

#### Example (using `gatttool` style pseudo commands)
```
# Write speed multiplier 1.5
char-write-req <speed_handle> 31 2e 35
# Write pattern 3
char-write-req <pattern_handle> 33
```
(Handles depend on discovery; use a BLE scanner to locate them.)

#### Reconnection / Advertising Notes
If the device stopped showing up after the first disconnect previously, this was due to relying on implicit auto-restart of advertising. The firmware now:
1. Registers `ServerCallbacks` during `BLEConfigServer::begin()`.
2. Explicitly restarts advertising in `onDisconnect` regardless of reason code.
3. Emits a status notification so you can verify the lifecycle in any BLE client that subscribes to the Status characteristic.

Typical disconnect reason codes (NimBLE / host stack dependent):

If you still do not see the device advertising:

##### Built-in Recovery Commands (via COMMAND characteristic)
Write ASCII tokens (no arguments yet) to the Command characteristic:
- `BLEADV`  – Force an advertising restart if no active connection.
- `BLEDROP` – Disconnect all known connections (if any) then restart advertising.

##### Advertising Watchdog
`BLEConfigServer::loop()` now runs a lightweight watchdog each main loop iteration:
1. If there are zero active connections.
2. And the advertising object reports `!isAdvertising()`.
3. It throttles and attempts a restart (status line `[BLE] ADV_RESTART reason=wd`).

This covers edge cases where the browser (Web Bluetooth) tab is refreshed or closed without performing a clean GATT disconnect, leaving the peripheral believing a connection is still pending until the supervision timeout. After the timeout / disconnect event, advertising is explicitly restarted.

### SandScript upload, runtime, and presets

- The GATT service now exposes a binary-safe **SandScript chunk characteristic** `9b6c7e18-3b2c-4d8c-9d7c-5e2a6d1f8b01` (write / write-without-response). Use it to stream plain-text script bytes (UTF-8).
- The command characteristic accepts the following tokens to orchestrate transfers:
  - `SCRIPT_BEGIN <numBytes> [patternIndex]` – reserve a transfer slot. `patternIndex` is optional; omit or use `0` to target the dedicated SandScript pattern (last slot).
  - `SCRIPT_DATA` – implicit for each write to the chunk characteristic (chunks can be any size up to negotiated MTU).
  - `SCRIPT_END` – finalize, compile, and activate the script. Status characteristic emits `[SCRIPT] READY len=<n>` and either `[SCRIPT] LOADED ...` or a `[SCRIPT] COMPILE_ERR ...` message.
  - `SCRIPT_ABORT` – cancel the current transfer.
  - `SCRIPT_STATUS` – echo the internal transfer state (helpful for debugging clients).
  - Transfers idle for 5 seconds automatically timeout (`[SCRIPT] ERR timeout ...`) and reset.
- Script compile / runtime errors surface as status lines with `PSG:` prefixes (straight from the compiler). The last compiled script drives the **SandScript pattern slot**, which is appended to the end of the pattern list (e.g., `Pattern 11` if ten stock patterns exist).
- On-device DSL quick reference:
  - Inputs: `radius` (cm), `angle` (degrees), `start` (1 on first step), `rev` (continuous revolutions), `steps` (evaluation counter), `time` (ms since script started).
  - Outputs: assign to `next_radius`, `next_angle`, `delta_radius`, `delta_angle` (all in cm / degrees). Locals are automatically permitted (`foo = ...`).
  - Functions: `sin`, `cos` (degrees in / unitless out), `abs`, `clamp(value, min, max)`, `sign`.
  - Operators: `+`, `-`, `*`, `/`, `%`, unary `-`. Comments start with `#` and run to end-of-line. One assignment per line; no semicolons needed.
- Presets bundled in firmware:
  1. **BloomSpiral** – gentle expanding spiral with periodic bloom (loaded automatically at boot).
  2. **LotusOrbit** – oscillating radial orbit with harmonic angle sweep.
- Use the command characteristic to manage presets:
  - `SCRIPT_PRESET` *(no args)* – load preset #1 (`BloomSpiral`).
  - `SCRIPT_PRESET <index>` – 1-based numeric index (e.g., `SCRIPT_PRESET 2`).
  - `SCRIPT_PRESET <name>` – case-insensitive name (`SCRIPT_PRESET lotusorbit`).
  - `SCRIPT_PRESET list` – list available presets in a single status line.
  - Successful loads automatically switch to the SandScript pattern (or force a restart if already selected).

## Web Visualization (Firmware-Accurate Patterns)

The file `tools/webclient-mockup.html` now includes a simulation engine that mirrors the firmware pattern generator logic for visual verification of geometry before running on hardware.

### Pattern parity mapping
| ID | Name | Notes on parity |
|----|------|-----------------|
|1|Simple Spiral|Identical incremental angle (360/100) & radial step inversion when bounds exceeded.|
|2|Cardioids|Implements 43° angular increments and 1/8 radial triangle-wave flipping direction at bounds.|
|3|Wavy Spiral|Adds sinusoidal radial offset (`amplitude=200`, `period=8`) to base spiral.|
|4|Rotating Squares|Edge-by-edge traversal with 10° rotation after each square; line interpolation reproduces `drawLine` origin proximity resolution bump & vertical-line handling.|
|5|Pentagon Spiral|Sequential edges; vertex radii adjusted by `radialStepover=500` with boundary inversion logic.|
|6|Hexagon Vortex|Hex edges traced; rotation (5° per cycle) and radius pulsation with same overshoot rule (`>= MAX_R_STEPS + 2000`).|
|7|Pentagon Rainbow|Off-center pentagon (base radius 3000) translated by radial 4000 and rotated/translated each iteration (`shiftDeg=2`).|
|8|Random Walk 1|Random absolute (radial, angular) targets each step, shortest-angle wrap implied by final absolute placement (no interpolation).|
|9|Random Walk 2|Random targets connected by straight lines via identical line iterator.|
|10|Accidental Butterfly|Spiral plus radial `200*sin(8θ)` and angular `40*cos(3θ)` offsets (converted to steps).|

### Shared math utilities ported
`convertDegreesToSteps`, `convertStepsToRadians`, `modulus`, `nGonGenerator`, `translatePoints`, and a stateful `drawLine` replica (with vertical-line rotation and near-origin densification) were translated from C++ to JavaScript. These ensure geometric equivalence to firmware outputs.

### Simulation model differences
1. Motion timing (speed scaling, coordination, acceleration) is not modeled—only the sequence of endpoint target positions.
2. Interruptibility (e.g., early stop mid-move) is not simulated; each target is treated as if reached.
3. Random walks are non-deterministic; to compare runs deterministically, a seeded RNG could be added later.

### Tuning
Each entry in `patternMeta` has a `pointsTarget` controlling how many target steps are simulated. Adjust for more/less detail. A safety limiter aborts if loop iterations exceed `4 * pointsTarget` to avoid infinite loops if future pattern changes mis-handle termination.

### Updating after firmware changes
When modifying a pattern in `sand-garden.ino`, replicate the change in the corresponding factory inside `generatePatternPoints()` to keep the visualization in sync.

### Future enhancements (suggested)
- Optional real-time stepwise animation driven by the same pattern function (instead of precomputing all points).
- Visual speed normalization (shading or dash length by implied travel time).
- Overlay of live telemetry path vs. predicted pattern.

## Ball + Trench Rendering (Sand Displacement Simulation)

The browser visualizer now simulates a spherical ball carving a shallow trench rather than drawing a single-pixel polyline.

### Implementation Overview
Inside `tools/webclient-mockup.html` the following additions were made:
1. Offscreen `trenchCanvas` (same dimensions as the main canvas) accumulates a pseudo height/depth map.
2. During `updateTrail()` every sampled point along each finished segment stamps two layered radial gradients:
  - A dark multiply gradient (core depression) with soft falloff (`ballRadiusPx + trenchSoftness`).
  - A faint lighter gradient offset slightly up/left for a subtle raised lip highlight.
3. In `renderFrame()` the trench canvas is composited onto the sand background using `globalCompositeOperation = 'multiply'` before overlaying the (optional) thin path reference canvas.
4. The moving ball head renders with: under-shadow (multiply radial gradient), body radial gradient (cool-to-warm whites), and a small specular highlight.

### Key Tunables
Located near the top of the Alpine component state:
```
ballRadiusPx: 6         // Visual radius of the rolling ball (px)
trenchDepth: 1.0        // Reserved for possible normal map scaling (currently aesthetic only)
trenchSoftness: 8       // Additional blur radius around the ball imprint
```
Adjusting `ballRadiusPx` automatically changes stamping sample density (spacing ≈ 0.9 * radius) to keep grooves continuous without overdraw.

### Performance Notes
- All stamping uses simple `createRadialGradient` fills; no per-frame convolution passes.
- Composite operations switch among `multiply`, `lighter`, then back to `source-over` each stamp. Browsers batch these reasonably well for moderate point counts (< ~10k).
- If large pattern counts cause frame drops, reduce samples per segment (increase divisor in calculation) or shrink `trenchSoftness`.

### Future Ideas
- Generate a normal map from the accumulated depth for more realistic directional lighting (single pass Sobel filter on trench buffer).
- Add configurable lighting direction & intensity controls.
- Allow fade/erode of old trenches for a "resetting sand" effect.
- Support importing real telemetry points to carve in real-time instead of simulated precomputed path.

### Update: Real-Time Incremental Trench (Replaces Polyline)
The trench is now carved progressively as the ball advances each animation frame. Instead of drawing a partial polyline for the current segment:
1. The previous head position is cached (`lastHeadX/lastHeadY`).
2. The segment between last and current head location is subdivided at spacing `ballRadiusPx * trenchStampSpacingFactor`.
3. Each subdivision stamps a dual-gradient depression + highlight into `trenchCanvas` (no separate path polyline).
4. The updated trench texture is re-blended immediately (multiply) so the visible groove matches the ball’s position with no delayed reveal.

`updateTrail()` is now effectively a fast index advance so large simulation jumps don’t backlog stamping; all visual carving happens in `renderFrame()`.


