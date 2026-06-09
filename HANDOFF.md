# Teensy Handoff — QuantumStrike Motion Pipeline
*Last updated: 2026-06-08 (Session 4)*

---

## Session History

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
| `src/TrajectoryExecutor.h` | S3: telemetry struct, PKT_TYPE_TELEMETRY, Switch&. S4: `serialLatencyUs` in struct, `measuredLatencyUs_` member. S5: SPI include, mouse constants, `vxMouseMs_`/`vyMouseMs_`/`lastMouseUs_` members, mouse method declarations |
| `src/TrajectoryExecutor.cpp` | S3: sendTelemetry(), telemetry timer, Switch&. S4: handleClockPong saves latency, gyro stubs replaced. S5: `initMouse()`, `updateMouseVelocity()`, `pmwRead()`, `pmwWrite()` implemented; `readMouseVx/Vy()` return cached values |
| `src/main.cpp` | Complete — passes `switches` to TrajectoryExecutor constructor |
| `src/Motor.cpp/h` | Unchanged |
| `src/Movement.cpp/h` | Unchanged — still used for line avoidance |
| `src/Cam.cpp/h` | Unchanged — CamCalc no longer called; file kept for reference |
| `src/CompassSensor.cpp/h` | S4: `getOmegaRadS()` added — BNO055 VECTOR_GYROSCOPE query, negated for CW convention |
| `src/Defense.cpp/h` | Unchanged |

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
    29     3  _pad             uint8_t   — alignment
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
TrajectoryExecutor trajectoryExecutor(FL, FR, BL, BR, compassSensor, switches);

// runOffense() call order:
lineDetection.Calculate();
// camera.CamCalc() REMOVED — Pi no longer sends ASCII camera data
lineAngle  = lineDetection.getAngle();
orbitAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);
movement.kickBackground();
trajectoryExecutor.processSerial();   // drain Serial2 + send ping/telemetry every loop

if (!switches.start()) { movement.stop(); return; }
if (lineAngle != -5) { /* line avoidance */ return; }
if (switches.lightgate() && fabs(goalAngle) < 5) { movement.kick(); return; }

if (!trajectoryExecutor.execute()) {
    if (camera.ballAngle != -5) { movement.movement(...); }
    else { movement.stop(); }
}
```

---

## Remaining TODOs

| Priority | Item | Location | What to do |
|----------|------|----------|------------|
| High | Wheel angles | `TrajectoryExecutor.cpp` `ALPHA_WHEEL[]` | Physically verify against mechanical drawing. Values FR=−0.6109, RR=+0.6109, RL=+2.5307, FL=−2.5307 rad — unconfirmed |
| Medium | Wire mouse sensor | `TrajectoryExecutor.h` `MOUSE_CS_PIN = -1` | Set `MOUSE_CS_PIN` to the Teensy SPI CS pin; verify `MOUSE_CPI` formula for your PMW3389 firmware; confirm dx=+right/dy=+forward in body frame |
| Medium | Battery voltage | `readBatteryVoltage()` | Wire resistor divider to Teensy ADC |
| Low | Motor constants kS/kV/kA | Top of `TrajectoryExecutor.cpp` | Bench-characterise via voltage step response. Placeholders: 0.5 / 0.08 / 0.02 |
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
