# Teensy Handoff — QuantumStrike Motion Pipeline
*Last updated: 2026-06-09 (Session 8)*

---

## Session History

### Session 8 — Kicker and dribbler control signals added to ActionChunk

- **Wire format**: Reused 2 of the 3 existing `_pad` bytes in the `ActionChunk` header. `kick` (offset 29) and `dribblerPower` (offset 30) replace `_pad[0..1]`; `_pad` is now 1 byte. Actions array still starts at offset 32 — struct size unchanged.
- **Pi owns kick**: Removed the local `switches.lightgate() && fabs(goalAngle) < 5` kicker check from `main.cpp`. Pi sends `kick=1` in a chunk when `hasBall && latestDebug_.usedStrikePosePlan` (NormalStrike committed with ball secured). Dribbler runs at power 200/255 whenever offense is active, off when idle.
- **Offense2026 new/modified files**:
  - `Movement.h/.cpp` — added `static constexpr int DRIBBLER_PIN = -1` (TODO: assign), `setDribbler(uint8_t power)` method (`analogWrite` guarded by pin ≥ 0), and `analogWriteFrequency` + `pinMode` in constructor.
  - `TrajectoryExecutor.h` — `ActionChunk._pad[3]` → `kick + dribblerPower + _pad(1B)`; added `#include <Movement.h>`, `Movement& movement_` member, updated constructor signature.
  - `TrajectoryExecutor.cpp` — constructor takes `Movement& movement`; in `execute()` at chunk hot-swap: calls `movement_.kick()` if `chunk.kick`, then `movement_.setDribbler(chunk.dribblerPower)`.
  - `main.cpp` — `TrajectoryExecutor` constructor now passes `movement`; local kicker block removed.
- **BallAlgo (Pi) modified files**:
  - `motion/Protocol.hpp` — `packActionChunk` signature gains `uint8_t kick = 0, uint8_t dribblerPower = 0`.
  - `motion/Protocol.cpp` — writes `kick` and `dribblerPower` at offsets 29/30; `off += 1` (was `off += 3`).
  - `motion/ActionChunkPublisher.cpp` — computes `kick`/`dribbler` after `debugPlan`; idle-stop path explicitly passes `0, 0`.

### Session 7 — LinePCB split: line sensors + mouse sensor moved to separate Teensy 4.0

- **New architecture**: `LinePCBCode2026` is a new PlatformIO project for a Teensy 4.0 that owns all line sensors (MCP3008 ADCs via SPI) and the PMW3389 mouse sensor (SPI). It sends processed data to the main Teensy 4.1 over **Serial2 @ 1 Mbaud** (4.0 TX2/RX2 ↔ 4.1 TX2/RX2).
- **Pi serial port moved**: Pi ↔ Teensy 4.1 communication shifted from `Serial2` to **`Serial3`** (`TrajectoryExecutor` updated throughout). Wire: Pi TX → Teensy 4.1 RX3 (pin 15), Pi RX ← Teensy 4.1 TX3 (pin 14).
- **LinePCBCode2026 new files**:
  - `src/LinePCBController.h/.cpp` — `LinePCBController` class: runs `LineDetection::Calculate()` and PMW3389 `updateMouse()` every loop, sends `LinePCBDataPkt` each loop and `LinePCBDebugPkt` when debug flag is set, handles `LinePCBCmdPkt` commands from main Teensy (debug enable + calibration trigger).
  - `src/LineDetection.h/.cpp`, `src/trig.h/.cpp` — copied verbatim from Offense2026; no changes.
  - `src/main.cpp` — trivial: constructs `LinePCBController(Serial2)`, calls `begin(1000000)` / `loop()`.
  - `platformio.ini` — board changed to `teensy40`; lib_deps: `bakercp/MCP3XXX`.
- **Offense2026 new files**:
  - `src/LinePCBComm.h/.cpp` — `LinePCBComm` class: drains Serial2, parses `LinePCBDataPkt` / `LinePCBDebugPkt`, caches `lineAngle`, `avoidanceAngle`, `mouseVx/Vy`, `chordLength`, `crossLine`, `activatedVals[48]`. Provides `setDebugEnabled(bool)` and `triggerCalibration()` which send `LinePCBCmdPkt`.
- **Offense2026 modified files**:
  - `TrajectoryExecutor.h/.cpp` — `Serial2` → `Serial3` everywhere; all PMW3389 SPI code removed (`initMouse`, `updateMouseVelocity`, `pmwRead`, `pmwWrite`, `MOUSE_CS_PIN`, `MOUSE_CPI`, `lastMouseUs_`, `#include <SPI.h>`); added public `setMouseVelocity(float vx, float vy)`.
  - `LcdController.h/.cpp` — constructor `LineDetection&` replaced with `LinePCBComm&`; `sendLineArray()` reads from `_linePCBComm.getActivatedVals()` instead of direct array access. Debug sensor data is populated when the main Teensy has `setDebugEnabled(true)` active.
  - `Callibration.h` / `Calibration.cpp` — `LineDetection&` dependency removed; `calibrateLineSensors()` removed (now handled by `LinePCBCmdPkt::calibrate` on the LinePCB side). Only `calibrateCompassSensor()` remains.
  - `main.cpp` — `LineDetection lineDetection` removed; `LinePCBComm linePCBComm(Serial2)` added; `lcdController` constructor updated; `Serial3.begin(2000000)` replaces `Serial2.begin(2000000)`; `linePCBComm.begin(1000000)` added in `setup()`; `loop()` calls `linePCBComm.update()` and `trajectoryExecutor.setMouseVelocity(...)` before anything else; line angle/avoidance read from `linePCBComm`.

### Session 6 — LCD logic extracted into LcdController
- All LCD-related code removed from `main.cpp` (~200 lines) and moved to `src/LcdController.h` / `src/LcdController.cpp`
- `RobotMode`, `LcdStartMode`, `LcdStartPosition` enums and `LcdControlState` struct moved to `LcdController.h`
- `LcdController` class takes constructor references to `HardwareSerial`, `LineDetection`, `CompassSensor`, `Switch`, `Movement`, and `RobotMode` — consistent with the existing module pattern
- Public API: `begin(baud)`, `readCommands()`, `sendTelemetry(lineAngle, avoidanceAngle)`, `sendCalibrationStatus()`, `applyRobotModeSettings()`, `isStartEnabled()`, `isGoalBlueSelected()`, `state` (LcdControlState)
- `lineAngle` / `avoidanceAngle` are now passed as parameters to `sendTelemetry()` rather than read as globals
- `main.cpp` instantiates `LcdController lcdController(Serial8, lineDetection, compassSensor, switches, movement, kRobotMode)` and calls its methods in place of the old free functions

### Session 1 — TrajectoryExecutor created
- `src/TrajectoryExecutor.h` and `src/TrajectoryExecutor.cpp` written (Pipeline.md Steps 7 & 8)
- `main.cpp` integration deferred

### Session 2 — Protocol unified, integration complete
- Full protocol audit against the Pi's `BallAlgo/src/motion/Protocol.cpp` found the two sides were completely incompatible (wrong magic, wrong CRC algorithm, wrong type codes, mismatched struct layout)
- Protocol corrected to match Pi exactly
- `camera.CamCalc()` removed — Pi no longer sends ASCII camera stream; Teensy receives binary ActionChunks only
- `TrajectoryExecutor` wired into `main.cpp`

### Session 5 — PMW3389 mouse sensor SPI driver
- `TrajectoryExecutor.h`: `#include <SPI.h>` added; `MOUSE_CS_PIN = -1` (TODO placeholder) and `MOUSE_CPI = 1600.f` (TODO: verify) constants added
- `pmwRead()` / `pmwWrite()` SPI helpers implemented (SPI Mode 3, 2 MHz, respects tSRAD/tSRR/tSWW timing)
- `initMouse()`: no-op when `MOUSE_CS_PIN < 0`; otherwise configures CS pin, calls `SPI.begin()`, runs PMW3389 power-up reset (0x3A → 0x5A), drains motion registers, writes CPI to registers 0x0F/0x10
- `updateMouseVelocity()`: PMW3389 Motion_Burst read (address 0x16, 12-byte burst); converts signed 16-bit dx/dy counts → m/s via `(counts × 25.4 / CPI) / (1000 × dt_s)`; stores in `vxMouseMs_` / `vyMouseMs_`; no-op when no motion flag or pin unset
- `readMouseVx()` / `readMouseVy()` now return cached `vxMouseMs_` / `vyMouseMs_`
- `processSerial()` calls `updateMouseVelocity()` every iteration (after telemetry timer, before serial drain)
- Three TODOs remain before live use: assign `MOUSE_CS_PIN`, verify CPI register formula for firmware version, verify dx/dy axis polarity against robot body frame

### Session 4 — Gyro omega wired; serial latency auto-calibration
- `CompassSensor::getOmegaRadS()` added — queries BNO055 via `VECTOR_GYROSCOPE` event, returns `-gyroEvent.gyro.z` (negated: BNO055 is CCW-positive, system convention is CW-positive)
- Both `omegaRadS = 0.0f` stubs replaced with `imu.getOmegaRadS()`: Coriolis feedforward in `execute()` is now live, and `sendTelemetry()` sends real gyro data to Pi EKF
- `TeensyTelemetryPayload` struct extended: `uint8_t _pad[3]` replaced with `uint16_t serialLatencyUs` + `uint8_t _pad1` (struct remains 20 bytes, no framing change)
- `TrajectoryExecutor` gains `uint16_t measuredLatencyUs_` member (private, initialised to 0)
- `handleClockPong()` now saves the computed `latency_us` into `measuredLatencyUs_` after each successful pong — first valid measurement latches it
- `sendTelemetry()` writes `p.serialLatencyUs = measuredLatencyUs_`; Pi reads this once at startup to calibrate `kPipelineLatencyUs` dynamically

### Session 3 — Beam-break telemetry (type 0x04), hasBall pipeline
- Added type `0x04` (Telemetry) to the protocol — Teensy → Pi at 100 Hz
- `TrajectoryExecutor` now accepts `Switch& sw`; `sendTelemetry()` packs heading, mouse vx/vy, gyro omega, and beam-break state into a 31-byte framed packet sent every `TELEMETRY_INTERVAL_MS = 10 ms`
- `processSerial()` fires telemetry timer alongside the existing ping timer
- `main.cpp` updated: `TrajectoryExecutor trajectoryExecutor(FL, FR, BL, BR, compassSensor, switches)`
- Pi side wired to receive type 0x04: `ClockSync` routes to `RobotSerial::cacheTeensyTelemetry()`, which feeds `MotionPlanner` for the `hasBall && NormalStrike` ball-retention cost path
- Fixed Pi-side serial drain bug: old `pollOdometry()` was calling `consumeAscii()` → `readSome()` every camera frame, consuming binary bytes before `ActionChunkPublisher` could process them; fix returns cached data without touching the port

---

## Current File State

| File | Status |
|------|--------|
| `src/TrajectoryExecutor.h` | S8: `ActionChunk` gains `kick`+`dribblerPower`; `Movement&` added to constructor. S7: SPI/PMW3389 removed; `Serial3`; `setMouseVelocity()`. S3–S5: telemetry, clock sync, Switch&. |
| `src/TrajectoryExecutor.cpp` | S8: constructor takes `Movement&`; chunk hot-swap fires `kick`/`setDribbler`. S7: `Serial3` throughout; SPI code removed. |
| `src/Movement.h` | S8: `DRIBBLER_PIN = -1` TODO constant; `setDribbler(uint8_t)` declaration added. |
| `src/Movement.cpp` | S8: `setDribbler()` implemented; dribbler `pinMode`+`analogWriteFrequency` in constructor. |
| `src/LinePCBComm.h` | S7: new — `LinePCBComm` class, protocol types, parser |
| `src/LinePCBComm.cpp` | S7: new — `update()`, `onPacket()`, `sendCommand()`, getters |
| `src/LcdController.h` | S7: `LinePCBComm&` replaces `LineDetection&`. S6: full class. |
| `src/LcdController.cpp` | S7: `sendLineArray()` reads from `_linePCBComm.getActivatedVals()`. |
| `src/Callibration.h` | S7: `LineDetection` dependency removed; only compass calibration remains. |
| `src/Calibration.cpp` | S7: `calibrateLineSensors()` removed; constructor takes only `CompassSensor&`. |
| `src/main.cpp` | S7: `LineDetection` removed; `LinePCBComm linePCBComm(Serial2)` added; `Serial3` for Pi; `loop()` calls `update()`+`setMouseVelocity()`. |
| `src/Motor.cpp/h` | Unchanged |
| `src/Movement.cpp/h` | Unchanged |
| `src/Cam.cpp/h` | Unchanged — CamCalc no longer called; file kept for reference |
| `src/CompassSensor.cpp/h` | S4: `getOmegaRadS()` added |
| `src/Defense.cpp/h` | Unchanged |

### LinePCBCode2026 File State

| File | Status |
|------|--------|
| `src/LinePCBController.h` | S7: new — protocol types, `LinePCBController` class |
| `src/LinePCBController.cpp` | S7: new — line+mouse loop, packet send/recv, calibration trigger |
| `src/LineDetection.h/.cpp` | S7: copied from Offense2026 — no changes |
| `src/trig.h/.cpp` | S7: copied from Offense2026 — no changes |
| `src/main.cpp` | S7: new — trivial `LinePCBController(Serial2)` setup/loop |
| `platformio.ini` | S7: `board=teensy40`, `lib_deps: bakercp/MCP3XXX` |

---

## Serial Protocol (Pi ↔ Teensy, Serial2 @ 2 Mbaud)

Binary framing — **matches Pi's `Protocol.cpp` exactly**:

```
┌──────────────────┬──────┬─────────────┬──────────────────┬─────────────┐
│ magic (4B LE)    │ type │ len (2B LE) │ payload (N bytes)│ CRC-32 (4B) │
└──────────────────┴──────┴─────────────┴──────────────────┴─────────────┘

Magic bytes (LE order): 0xFE 0xED 0xFA 0xCE  (= uint32 0xCEFAEDFE)
CRC-32/IEEE (poly 0xEDB88320) computed over [magic | type | len | payload]
```

| Direction | Type | Value | Payload |
|-----------|------|-------|---------|
| Teensy → Pi | Ping | `0x01` | 8 B — Teensy `t0` as `uint64_t µs` LE |
| Pi → Teensy | Pong | `0x02` | 16 B — `t0_echo uint64_t` + `t1_pi uint64_t` LE |
| Pi → Teensy | ActionChunk | `0x03` | variable — see ActionChunk layout below |
| Teensy → Pi | Telemetry | `0x04` | 20 B — `TeensyTelemetryPayload` struct |

### Previous (incorrect) protocol — do not use
The first session documented a different protocol (`0xAB 0xCD` magic, CRC-16 over payload only, type codes in a different order). That spec was never implemented on the Pi. The above is canonical on both sides.

---

## ActionChunk Wire Layout

```
Offset  Size  Field
     0     8  trajectory_id    uint64_t  — monotonically increasing
     8     8  start_time_pi    uint64_t  — Pi steady_clock µs when actions[0] executes
    16     2  dt_ms            uint16_t  — time step per slot (e.g. 4 ms)
    18     2  num_actions      uint16_t  — valid entries in actions[] (0 = stop)
    20     4  vx_meas          float     — robot body-frame vx at planning time (m/s)
    24     4  vy_meas          float     — robot body-frame vy at planning time (m/s)
    28     1  pose_valid       uint8_t   — 1 if Pi's lidar pose was valid
    29     1  kick             uint8_t   — 1 = fire kicker when this chunk starts executing
    30     1  dribblerPower    uint8_t   — 0–255 PWM dribbler power (0 = off)
    31     1  _pad             uint8_t   — alignment
    32  24×N  actions[N]       GlobalAction × num_actions
```

Each `GlobalAction` is 24 bytes: `vx_global, vy_global, omega, ax_global, ay_global, alpha` (all `float`).
Max `num_actions = 50`. Max payload = 32 + 50×24 = **1232 bytes**. `num_actions = 0` = idle/stop.

---

## Telemetry Packet Layout (type 0x04)

Sent by Teensy → Pi every `TELEMETRY_INTERVAL_MS = 10 ms` (100 Hz). Total frame = **31 bytes**.

```cpp
// src/TrajectoryExecutor.h
#pragma pack(push, 1)
struct TeensyTelemetryPayload {
    float    headingDeg;       // BNO055 absolute heading (deg)
    float    mouseVxBodyMmS;   // mouse X body-frame (mm/s, +right)    [stub: 0]
    float    mouseVyBodyMmS;   // mouse Y body-frame (mm/s, +forward)  [stub: 0]
    float    omegaRadS;        // gyro Z rate (rad/s, CW positive)     — LIVE via getOmegaRadS()
    uint8_t  hasBall;          // beam-break: 1 = ball in intake
    uint16_t serialLatencyUs;  // one-way serial latency from handleClockPong (µs); 0 until first pong
    uint8_t  _pad1;
};  // 20 bytes
#pragma pack(pop)
```

Fields currently stubbed to 0: `mouseVxBodyMmS`, `mouseVyBodyMmS`. `omegaRadS` is now live (BNO055 gyro Z via `CompassSensor::getOmegaRadS()`). `serialLatencyUs` is populated by `handleClockPong()` and used by the Pi to calibrate `kPipelineLatencyUs` at startup.

The Pi uses the telemetry data as follows:
- `headingDeg` / `mouseVx/Vy` / `omegaRadS` → EKF Step 1a dead-reckoning predict
- `hasBall` → gates the ball-retention A* cost (`ballRetention = hasBall && NormalStrike`)

---

## Clock Synchronisation (Step 5)

1. `processSerial()` calls `sendClockPing()` every **100 ms**
2. Ping frame: `[magic][0x01][8 LE][t0_us uint64_t][CRC-32]`
3. Pi reflects a Pong: `[magic][0x02][16 LE][t0_echo uint64_t][t1_pi uint64_t][CRC-32]`
4. `handleClockPong()` computes NTP-style offset:

```
latency_us      = (t2_teensy - t0_echo) / 2
clock_offset_us = t1_pi - t0_echo - latency_us
measuredLatencyUs_ = (uint16_t)latency_us   ← saved for next telemetry packet
```

5. Chunk execution time: `t_start_teensy = chunk.start_time_pi - clock_offset_us`

**Fallback:** while `clock_offset_us == 0` (no pong yet), chunks execute immediately on arrival.

`measuredLatencyUs_` is included in every subsequent telemetry packet as `serialLatencyUs`. The Pi reads it once at startup (first nonzero value) and uses it to set `pipelineLatencyUs_ = serialLatencyUs + kSerialLatencyMarginUs` (see Pi HANDOFF).

---

## main.cpp Integration

```cpp
// Global:
TrajectoryExecutor trajectoryExecutor(FL, FR, BL, BR, compassSensor, switches, movement);

// runOffense() call order:
movement.kickBackground();             // manage kicker solenoid hold/release
trajectoryExecutor.processSerial();    // drain Serial3 + send ping/telemetry every loop

if (!switches.start()) { movement.stop(); return; }
if (lineAngle != -5) { /* line avoidance */ return; }
// NOTE: local kicker check removed — Pi sends kick=1 in ActionChunk when NormalStrike+hasBall

if (!trajectoryExecutor.execute()) {
    if (camera.ballAngle != -5) { movement.movement(...); }
    else { movement.stop(); }
}
// On chunk hot-swap, TrajectoryExecutor calls movement.kick() and movement.setDribbler()
// based on the kick/dribblerPower fields stamped by the Pi.
```

---

## Remaining TODOs

| Priority | Item | Location | What to do |
|----------|------|----------|------------|
| High | Assign dribbler PWM pin | `src/Movement.h` `DRIBBLER_PIN = -1` | Set `DRIBBLER_PIN` to the Teensy 4.1 pin wired to the dribbler motor controller; verify PWM frequency (currently 20 kHz) suits the controller |
| High | Wire mouse sensor CS pin | `LinePCBCode2026/src/LinePCBController.h` `MOUSE_CS_PIN = -1` | Set `MOUSE_CS_PIN` to the Teensy 4.0 pin wired to PMW3389 NCS; verify `MOUSE_CPI` formula; confirm dx=+right/dy=+forward |
| High | Verify Pi serial wiring | Hardware | Pi TX → Teensy 4.1 **RX3** (pin 15); Pi RX ← Teensy 4.1 **TX3** (pin 14). Previously Serial2 pins 7/8. |
| High | Wheel angles | `TrajectoryExecutor.cpp` `ALPHA_WHEEL[]` | Physically verify against mechanical drawing. Values FR=−0.6109, RR=+0.6109, RL=+2.5307, FL=−2.5307 rad — unconfirmed |
| Medium | Enable LCD debug | `main.cpp` | Call `linePCBComm.setDebugEnabled(true)` when `lcdController` is in debug mode so `activatedVals` stream to LCD |
| Medium | Battery voltage | `readBatteryVoltage()` | Wire resistor divider to Teensy 4.1 ADC |
| Low | Motor constants kS/kV/kA | Top of `TrajectoryExecutor.cpp` | Bench-characterise via voltage step response. Placeholders: 0.119 / 5.35 / 0.17 |
| Low | Motor index→wiring | `executeAsymmetricDrive()` | Verify FR=0, BR=1, BL=2, FL=3 against physical wiring |

---

## Key Design Decisions & Gotchas

### Motor::setSpeed range
`Motor::setSpeed(double speed)` expects **[-1, 1]**. `executeAsymmetricDrive` passes `voltage / v_bus`.

### Frame convention (global)
- `+X` = East, `+Y` = North; `θ` = clockwise from North (BNO055 NDOF default)
- Global→local: `vx_local = vx_global·cos(θ) − vy_global·sin(θ)`

### IMU convention
`CompassSensor::getOrientation()` returns degrees, 0 = North, 90 = East, CW positive.

### Coriolis feedforward
```cpp
ax_local_total = ax_abs_local - omega_actual * vy_actual;
ay_local_total = ay_abs_local + omega_actual * vx_actual;
```
`omega_actual` is now live via `imu.getOmegaRadS()`. Sign convention: CW positive (negated from BNO055 raw). Verify sign on hardware — if Coriolis correction drives the robot in the wrong direction, negate `getOmegaRadS()` return value.

### Grace period
When the active chunk runs out of actions, `execute()` holds the last velocity for **20 ms** before `emergencyBrake()`. This is the window for the next chunk to arrive.

### Trajectory ID ordering
Out-of-order / stale packets (ID ≤ active chunk ID) are silently dropped in `onChunkReceived()`. The Pi's `planner_.nextTrajId()` is the single monotonic counter shared between idle and active chunks.

### Parser robustness
The 4-byte magic matching handles partial matches correctly. The running CRC covers `[magic|type|len|payload]` incrementally — no buffer copy needed.
