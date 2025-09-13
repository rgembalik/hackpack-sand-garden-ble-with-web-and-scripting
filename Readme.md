
## Changes since original

- 
- Output pin number for A0 is not correct for some reason on ESP32. I had to specify `GPIO_NUM_1` manually for that pin.
- Joystick analog reads did not work as expected on ESP32 — the resting state was around 75% of the max value. The original Arduino Nano is 5V, but the ESP32 uses 3.3V logic. Since the joystick gets its VCC from USB-C (typically 5V), this can **overdrive the ESP32 analog pins**. Technically, you should use a voltage divider to safely drop the joystick output to 3.3V for the ESP32. For quick testing, I applied a naive offset in code, but **for long-term reliability, add a voltage divider on each analog line**.

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

#### Example (using `gatttool` style pseudo commands)
```
# Write speed multiplier 1.5
char-write-req <speed_handle> 31 2e 35
# Write pattern 3
char-write-req <pattern_handle> 33
```
(Handles depend on discovery; use a BLE scanner to locate them.)