#ifndef TRAJECTORY_EXECUTOR_H
#define TRAJECTORY_EXECUTOR_H

#include <Arduino.h>
#include <Switches.h>
#include <Motor.h>
#include <CompassSensor.h>
#include <Movement.h>

// Mouse velocity is now supplied externally via setMouseVelocity() from LinePCBComm.

// â”€â”€â”€ Packed structs â€“ must match Pi serialisation byte-for-byte â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Both platforms are little-endian (ARM), raw memcpy is safe.
#pragma pack(push, 1)

struct GlobalAction {
    float vx_global;  // m/s, field +x (East)
    float vy_global;  // m/s, field +y (North)
    float omega;      // rad/s, clockwise positive
    float ax_global;  // m/sÂ², feedforward
    float ay_global;
    float alpha;      // rad/sÂ², feedforward
};

// Wire layout matches Pi’s packActionChunk exactly (Protocol.cpp):
//   [trajectory_id 8][start_time_pi 8][dt_ms 2][num_actions 2]
//   [vx_meas 4][vy_meas 4][pose_valid 1][kick 1][dribblerPower 1][_pad 1]
//   [actions[50] × 24]  →  total 1232 bytes
struct ActionChunk {
    uint64_t     trajectory_id;
    uint64_t     start_time_pi;  // Pi absolute timestamp (Âµs) for actions[0]
    uint16_t     dt_ms;          // time step per slot (e.g. 4 ms)
    uint16_t     num_actions;    // valid entries in actions[]
    float        vx_meas;        // robot body-frame measured velocity at planning time (m/s)
    float        vy_meas;
    uint8_t      pose_valid;     // 1 = Pi's lidar pose was valid when this chunk was planned
    uint8_t      kick;           // 1 = fire kicker when this chunk starts executing
    uint8_t      dribblerPower;  // 0-255 PWM dribbler power (0 = off)
    uint8_t      _pad;           // alignment — matches Pi padding in packActionChunk
    GlobalAction actions[50];    // max 50 = kChunkMaxActions in Pi config
};

#pragma pack(pop)

// â”€â”€â”€ Serial framing constants â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//
// PROTOCOL (Pi â†” Teensy, Serial2 @ 2 Mbaud):
//
//   Frame layout:
//   â”Œâ”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”
//   â”‚ magic (4B LE)    â”‚ type â”‚ len (2B LE)    â”‚ payload (N B)   â”‚ CRC-32 (4B) â”‚
//   â””â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”´â”€â”€â”€â”€â”€â”€â”´â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”´â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”´â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”˜
//
//   Magic = 0xCEFAEDFE  â†’  LE bytes: 0xFE 0xED 0xFA 0xCE
//   CRC-32/IEEE (poly 0xEDB88320) over [magic|type|len|payload] â€“ same as Pi's Protocol.cpp
//
//   Packet types (matching Pi's kMsg* constants in Protocol.hpp):
//     0x01  Ping  â€“ Teensyâ†’Pi,    payload = uint64_t Teensy t0 (Âµs)          8 B
//     0x02  Pong  â€“ Piâ†’Teensy,    payload = uint64_t t0_echo + uint64_t t1  16 B
//     0x03  ActionChunk â€“ Piâ†’Teensy, payload = sizeof(ActionChunk)         1232 B
//
static constexpr uint8_t  PKT_MAGIC_BYTES[4] = {0xFE, 0xED, 0xFA, 0xCE};
static constexpr uint8_t  PKT_TYPE_PING  = 0x01;  // Teensy â†’ Pi
static constexpr uint8_t  PKT_TYPE_PONG  = 0x02;  // Pi     â†’ Teensy
static constexpr uint8_t  PKT_TYPE_CHUNK = 0x03;  // Pi     â†’ Teensy
static constexpr uint8_t  PKT_TYPE_TELEMETRY  = 0x04;  // Teensy -> Pi

static constexpr uint16_t MAX_PAYLOAD_LEN = sizeof(ActionChunk);  // 1232 bytes

// How often the Teensy sends clock-sync pings to the Pi.
static constexpr uint32_t PING_INTERVAL_MS = 100;

// Telemetry payload sent by Teensy -> Pi at TELEMETRY_INTERVAL_MS.
// Must match Pi's TeensyTelemetry struct in Protocol.hpp byte-for-byte.
#pragma pack(push, 1)
struct TeensyTelemetryPayload {
    float   headingDeg;      // BNO055 absolute heading (deg)
    float   mouseVxBodyMmS;  // mouse X body-frame (mm/s, +right)
    float   mouseVyBodyMmS;  // mouse Y body-frame (mm/s, +forward)
    float   omegaRadS;       // gyro Z rate (rad/s, CW positive)
    uint8_t hasBall;         // beam-break: 1 = ball in intake
    uint8_t startEnabled;    // LCD-adjusted start state
    uint8_t goalIsBlue;      // 1 = attack blue, defend yellow
    uint8_t modeOverride;    // 0 = auto, 1 = manual offense, 2 = manual defense
    uint16_t serialLatencyUs;  // one-way serial latency from handleClockPong (µs); 0 until first pong
    uint16_t reserved;
};
#pragma pack(pop)
static_assert(sizeof(TeensyTelemetryPayload) == 24, "TeensyTelemetryPayload must be 24 bytes");

static constexpr uint16_t TELEMETRY_PKT_LEN     = sizeof(TeensyTelemetryPayload);
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 10;  // 100 Hz odometry stream


class TrajectoryExecutor {
public:
    TrajectoryExecutor(Motor& FL, Motor& FR, Motor& BL, Motor& BR,
                       CompassSensor& imu, Switch& sw, Movement& movement);

    // Drain Serial3 (Pi link), run the packet framing state machine, and send a
    // clock-sync ping every PING_INTERVAL_MS.  Must be called every loop iteration.
    void processSerial();

    // Inject mouse velocity from LinePCBComm (called by main.cpp after update()).
    void setMouseVelocity(float vx, float vy);
    void setMatchState(bool startEnabled, bool goalIsBlue, uint8_t modeOverride);

    // Execute the time-indexed action from the active chunk.
    // Returns true while actively driving (nominal or grace-period hold).
    // Returns false when idle â€“ caller should fall back or stop.
    // Only call this when line avoidance is NOT active.
    bool execute();

    // Hard-stop all four motors immediately.
    void emergencyBrake();

    // â”€â”€ Velocity PID gains (pure feedforward until mouse sensor is live) â”€â”€â”€â”€â”€â”€
    float Kp_x     = 0.0f;
    float Kp_y     = 0.0f;
    float Kp_omega = 0.0f;

private:
    Motor&         FLMotor;
    Motor&         FRMotor;
    Motor&         BLMotor;
    Motor&         BRMotor;
    CompassSensor& imu;
    Switch&        sw;
    Movement&      movement_;

    // â”€â”€ Trajectory double-buffer â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    ActionChunk active_chunk;
    ActionChunk queued_chunk;
    uint64_t    t_start_active_us = 0;
    uint64_t    t_start_queued_us = 0;
    bool        has_queued_chunk  = false;
    bool        is_first_chunk    = true;

    // â”€â”€ Clock synchronisation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // clock_offset_us: pi_time_us â‰ˆ teensy_micros + clock_offset_us
    // Updated each time a valid Pong is received.
    int64_t  clock_offset_us = 0;
    uint64_t ping_t0_us      = 0;   // Teensy micros() when the last Ping was sent
    uint32_t last_ping_ms    = 0;   // millis() of last Ping transmission
    uint32_t last_telemetry_ms  = 0;  // millis() of last Telemetry transmission
    uint16_t measuredLatencyUs_ = 0;  // one-way serial latency from last handleClockPong

    // Mouse velocity injected from LinePCBComm via setMouseVelocity()
    float vxMouseMs_ = 0.0f;
    float vyMouseMs_ = 0.0f;
    bool startEnabled_ = false;
    bool goalIsBlue_ = true;
    uint8_t modeOverride_ = 0;

    int BATT_ADC_PIN = 17;
    int DIVIDER_RATIO = (30000 + 7500) / 7500

    // â”€â”€ Serial parse state machine â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    enum class ParseState : uint8_t {
        MAGIC,    // match 4 magic bytes (uses magic_idx)
        TYPE,
        LEN_LO,
        LEN_HI,
        PAYLOAD,
        CRC       // accumulate 4 CRC bytes (uses crc_idx)
    };
    ParseState parse_state = ParseState::MAGIC;
    uint8_t    magic_idx   = 0;     // which magic byte is expected next (0â€“3)
    uint8_t    pkt_type    = 0;
    uint16_t   pkt_len     = 0;
    uint16_t   payload_idx = 0;
    uint8_t    crc_idx     = 0;     // CRC bytes received so far (0â€“3)
    uint8_t    crc_buf[4]  = {};    // received CRC bytes (LE)
    uint32_t   running_crc = 0;     // incremental CRC-32 over [magic|type|len|payload]
    uint8_t    payload_buf[MAX_PAYLOAD_LEN];

    // â”€â”€ Private helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    void     onChunkReceived(const ActionChunk& chunk);
    void     handleClockPong(const uint8_t* payload);
    void     sendClockPing();
    void     sendTelemetry();
    void     executeAsymmetricDrive(float vx, float vy, float omega,
                                    float ax, float ay, float alpha_rot);
    float    soft_sign(float w, float epsilon = 0.2f);
    float    readBatteryVoltage();
    float    readMouseVx();
    float    readMouseVy();
    uint32_t crc32_update(uint32_t crc, uint8_t b);
    uint32_t crc32(const uint8_t* data, uint16_t len);
};

#endif // TRAJECTORY_EXECUTOR_H
