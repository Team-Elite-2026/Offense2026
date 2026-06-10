#include <TrajectoryExecutor.h>
#include <Movement.h>
#include <cmath>
#include <cstring>
#include <algorithm>

// â”€â”€â”€ Robot geometry â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Wheel axis offsets from North, converted to the kinematic alpha convention:
//   v_wheel_i = sin(alpha_i)*vx_local + cos(alpha_i)*vy_local - R_CHASSIS*omega
// where alpha = (physical_offset_deg - 90Â°) in radians.
//
// Wheel order throughout this file: [FR, RR, RL, FL]
static constexpr float ALPHA_WHEEL[4] = {
    -0.6109f,  // FR  (55Â°  - 90Â° = -35Â°)
     0.6109f,  // RR  (125Â° - 90Â° =  35Â°)
     2.5307f,  // RL  (235Â° - 90Â° = 145Â°)
    -2.5307f   // FL  (305Â° - 90Â° = -145Â°)
};
static constexpr float R_CHASSIS = 0.09f;   // m, centre-to-wheel rotation radius
static constexpr float R_WHEEL   = 0.025f;  // m, wheel rolling radius

// â”€â”€â”€ Motor feedforward model: V = kSÂ·sign(Ï‰) + kVÂ·Ï‰ + kAÂ·Î± â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// TODO: replace with bench-characterised values.
static constexpr float kS = 0.119f;   // V  â€“ static friction threshold
static constexpr float kV = 5.35f;  // VÂ·s/rad  â€“ back-EMF coefficient
static constexpr float kA = 0.17f;  // VÂ·sÂ²/rad â€“ inertia coefficient

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

TrajectoryExecutor::TrajectoryExecutor(Motor& FL, Motor& FR, Motor& BL, Motor& BR,
                                       CompassSensor& imu, Switch& sw, Movement& movement)
    : FLMotor(FL), FRMotor(FR), BLMotor(BL), BRMotor(BR), imu(imu), sw(sw), movement_(movement)
{
    memset(&active_chunk, 0, sizeof(active_chunk));
    memset(&queued_chunk, 0, sizeof(queued_chunk));
    memset(payload_buf,   0, sizeof(payload_buf));
    memset(crc_buf,       0, sizeof(crc_buf));
}

// â”€â”€â”€ CRC-32/IEEE (poly 0xEDB88320, init 0xFFFFFFFF, finalise with ~) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Matches Pi's crc32() in Protocol.cpp exactly.
uint32_t TrajectoryExecutor::crc32_update(uint32_t crc, uint8_t b) {
    crc ^= b;
    for (uint8_t k = 0; k < 8; ++k)
        crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    return crc;
}

uint32_t TrajectoryExecutor::crc32(const uint8_t* data, uint16_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint16_t i = 0; i < len; ++i) crc = crc32_update(crc, data[i]);
    return ~crc;
}

// â”€â”€â”€ Serial packet parsing â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Frame format: [magic 4B LE][type 1B][len 2B LE][payload N B][CRC-32 4B LE]
// CRC-32 is computed over [magic|type|len|payload] (the whole frame except CRC).
// running_crc is maintained incrementally as bytes arrive.
void TrajectoryExecutor::processSerial() {
    // Periodic clock-sync ping (Teensy â†’ Pi).
    uint32_t now_ms = millis();
    if (now_ms - last_ping_ms >= PING_INTERVAL_MS) {
        sendClockPing();
        last_ping_ms = now_ms;
    }

    // Periodic telemetry packet (Teensy -> Pi) at 100 Hz.
    if (now_ms - last_telemetry_ms >= TELEMETRY_INTERVAL_MS) {
        sendTelemetry();
        last_telemetry_ms = now_ms;
    }

    while (Serial3.available() > 0) {
        uint8_t b = (uint8_t)Serial3.read();

        switch (parse_state) {

        case ParseState::MAGIC:
            if (b == PKT_MAGIC_BYTES[magic_idx]) {
                // Start fresh CRC on the first magic byte; continue on the rest.
                if (magic_idx == 0) {
                    running_crc = crc32_update(0xFFFFFFFFu, b);
                } else {
                    running_crc = crc32_update(running_crc, b);
                }
                if (++magic_idx == 4) {
                    magic_idx  = 0;
                    parse_state = ParseState::TYPE;
                }
            } else {
                // Mismatch: restart.  If b is the first magic byte, begin matching.
                if (b == PKT_MAGIC_BYTES[0]) {
                    magic_idx  = 1;
                    running_crc = crc32_update(0xFFFFFFFFu, b);
                } else {
                    magic_idx = 0;
                }
            }
            break;

        case ParseState::TYPE:
            pkt_type    = b;
            running_crc = crc32_update(running_crc, b);
            parse_state = ParseState::LEN_LO;
            break;

        case ParseState::LEN_LO:
            pkt_len     = b;
            running_crc = crc32_update(running_crc, b);
            parse_state = ParseState::LEN_HI;
            break;

        case ParseState::LEN_HI:
            pkt_len    |= (uint16_t)b << 8;
            running_crc = crc32_update(running_crc, b);
            if (pkt_len == 0 || pkt_len > MAX_PAYLOAD_LEN) {
                parse_state = ParseState::MAGIC;
            } else {
                payload_idx = 0;
                parse_state = ParseState::PAYLOAD;
            }
            break;

        case ParseState::PAYLOAD:
            payload_buf[payload_idx++] = b;
            running_crc = crc32_update(running_crc, b);
            if (payload_idx >= pkt_len)
                parse_state = ParseState::CRC;
            break;

        case ParseState::CRC:
            crc_buf[crc_idx++] = b;
            if (crc_idx >= 4) {
                crc_idx = 0;

                uint32_t received;
                memcpy(&received, crc_buf, 4);
                uint32_t computed = ~running_crc;  // finalise

                if (received == computed) {
                    if (pkt_type == PKT_TYPE_CHUNK && pkt_len >= 32) {
                        // Zero the full struct first so any unfilled action slots are safe.
                        ActionChunk chunk;
                        memset(&chunk, 0, sizeof(chunk));
                        memcpy(&chunk, payload_buf, pkt_len);
                        onChunkReceived(chunk);
                    } else if (pkt_type == PKT_TYPE_PONG && pkt_len >= 16) {
                        handleClockPong(payload_buf);
                    }
                }
                parse_state = ParseState::MAGIC;
            }
            break;

        } // switch
    }
}

void TrajectoryExecutor::setMatchState(bool startEnabled, bool goalIsBlue, uint8_t modeOverride) {
    startEnabled_ = startEnabled;
    goalIsBlue_ = goalIsBlue;
    modeOverride_ = modeOverride;
}

// â”€â”€â”€ Chunk arrival handler â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void TrajectoryExecutor::onChunkReceived(const ActionChunk& chunk) {
    if (!is_first_chunk &&
        chunk.trajectory_id <= active_chunk.trajectory_id) {
        return;  // stale or duplicate
    }

    queued_chunk = chunk;

    if (clock_offset_us == 0) {
        // Clock sync not yet locked: execute immediately on arrival.
        t_start_queued_us = (uint64_t)micros();
    } else {
        t_start_queued_us = (uint64_t)((int64_t)chunk.start_time_pi - clock_offset_us);
    }

    has_queued_chunk = true;
}

// â”€â”€â”€ Clock sync: send Ping (Teensy â†’ Pi) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Frame: [magic 4B][PKT_TYPE_PING 1B][8 LE 2B][t0 uint64_t 8B][CRC-32 4B]
// --- Telemetry packet (Teensy -> Pi, type 0x04) ---------------------------------
// Packs heading, mouse vx/vy, gyro omega, and beam-break hasBall into the
// TeensyTelemetryPayload struct and sends a framed binary packet over Serial2.
void TrajectoryExecutor::sendTelemetry() {
    TeensyTelemetryPayload p;
    p.headingDeg     = (float)imu.getOrientation();
    p.mouseVxBodyMmS = readMouseVx() * 1000.0f;   // m/s -> mm/s
    p.mouseVyBodyMmS = readMouseVy() * 1000.0f;   // m/s -> mm/s
    p.omegaRadS      = imu.getOmegaRadS();
    p.hasBall         = sw.lightgate() ? 1u : 0u;
    p.startEnabled    = startEnabled_ ? 1u : 0u;
    p.goalIsBlue      = goalIsBlue_ ? 1u : 0u;
    p.modeOverride    = modeOverride_;
    p.serialLatencyUs = measuredLatencyUs_;
    p.reserved        = 0;

    // Frame: [magic 4B][type 1B][len 2B][payload 24B][CRC-32 4B] = 35 bytes
    constexpr uint16_t plen = TELEMETRY_PKT_LEN;
    uint8_t frame[4 + 1 + 2 + plen + 4];
    memcpy(frame + 0, PKT_MAGIC_BYTES, 4);
    frame[4] = PKT_TYPE_TELEMETRY;
    memcpy(frame + 5, &plen, 2);
    memcpy(frame + 7, &p, plen);
    uint32_t c = crc32(frame, 7 + plen);
    memcpy(frame + 7 + plen, &c, 4);
    Serial3.write(frame, sizeof(frame));
}

void TrajectoryExecutor::sendClockPing() {
    ping_t0_us = (uint64_t)micros();

    uint8_t frame[4 + 1 + 2 + 8 + 4];  // 19 bytes total
    memcpy(frame + 0, PKT_MAGIC_BYTES, 4);
    frame[4] = PKT_TYPE_PING;
    uint16_t plen = 8;
    memcpy(frame + 5, &plen, 2);
    memcpy(frame + 7, &ping_t0_us, 8);
    // CRC over [magic|type|len|payload] = first 15 bytes.
    uint32_t c = crc32(frame, 15);
    memcpy(frame + 15, &c, 4);
    Serial3.write(frame, sizeof(frame));
}

// â”€â”€â”€ Clock sync: receive Pong (Pi â†’ Teensy) and update clock_offset_us â”€â”€â”€â”€â”€â”€â”€
// Pong payload: [t0_echo uint64_t 8B][t1_pi uint64_t 8B]
// NTP-style: clock_offset_us = t1_pi - (t0 + round_trip/2)
void TrajectoryExecutor::handleClockPong(const uint8_t* payload) {
    uint64_t t0_echo, t1_pi;
    memcpy(&t0_echo, payload + 0, 8);
    memcpy(&t1_pi,   payload + 8, 8);

    if (t0_echo != ping_t0_us) return;  // stale pong from a previous ping

    uint64_t t2 = (uint64_t)micros();
    int64_t  latency_us = (int64_t)(t2 - ping_t0_us) / 2;
    clock_offset_us = (int64_t)t1_pi - (int64_t)ping_t0_us - latency_us;

    if (latency_us > 0 && latency_us < 65535)
        measuredLatencyUs_ = (uint16_t)latency_us;
}

// â”€â”€â”€ Main execution step (Pipeline.md Step 7) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
bool TrajectoryExecutor::execute() {
    uint64_t t_now_us = (uint64_t)micros();

    // â”€â”€ Hot-swap queued chunk when its scheduled start time has arrived â”€â”€â”€â”€â”€â”€â”€
    if (has_queued_chunk) {
        if (is_first_chunk || t_now_us >= t_start_queued_us) {
            active_chunk      = queued_chunk;
            t_start_active_us = t_start_queued_us;
            has_queued_chunk  = false;
            is_first_chunk    = false;
            // Apply actuator commands stamped on this chunk by the Pi.
            if (active_chunk.kick) movement_.kick();
            movement_.setDribbler(active_chunk.dribblerPower);
        }
    }

    if (is_first_chunk) return false;

    // 0-action chunks signal the Pi wants the robot stopped.
    if (active_chunk.dt_ms == 0 || active_chunk.num_actions == 0) {
        return false;
    }

    // â”€â”€ Time-indexed action lookup â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    uint64_t elapsed_us   = t_now_us - t_start_active_us;
    uint32_t action_index = (uint32_t)((elapsed_us / 1000u) / active_chunk.dt_ms);

    GlobalAction target;

    if (action_index >= active_chunk.num_actions) {
        // â”€â”€ Grace period: hold last velocity with zeroed feedforward â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        uint32_t grace_limit = active_chunk.num_actions
                             + (20u / active_chunk.dt_ms);
        if (action_index < grace_limit) {
            target           = active_chunk.actions[active_chunk.num_actions - 1];
            target.ax_global = 0.0f;
            target.ay_global = 0.0f;
            target.alpha     = 0.0f;
        } else {
            emergencyBrake();
            return false;
        }
    } else {
        target = active_chunk.actions[action_index];
    }

    // â”€â”€ Sensor reads â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    float theta_rad = (float)imu.getOrientation() * (float)M_PI / 180.0f;
    float vx_actual    = readMouseVx();
    float vy_actual    = readMouseVy();
    float omega_actual = imu.getOmegaRadS();

    // â”€â”€ 1. Global-to-local frame rotation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    float cos_th = cosf(theta_rad);
    float sin_th = sinf(theta_rad);

    float vx_local_target = target.vx_global * cos_th - target.vy_global * sin_th;
    float vy_local_target = target.vx_global * sin_th + target.vy_global * cos_th;

    float ax_abs_local = target.ax_global * cos_th - target.ay_global * sin_th;
    float ay_abs_local = target.ax_global * sin_th + target.ay_global * cos_th;

    // â”€â”€ 2. Coriolis feedforward compensation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    float ax_local_total = ax_abs_local - omega_actual * vy_actual;
    float ay_local_total = ay_abs_local + omega_actual * vx_actual;

    // â”€â”€ 3. Velocity PID correction â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    float vx_cmd    = vx_local_target + Kp_x     * (vx_local_target - vx_actual);
    float vy_cmd    = vy_local_target + Kp_y     * (vy_local_target - vy_actual);
    float omega_cmd = target.omega    + Kp_omega * (target.omega     - omega_actual);

    executeAsymmetricDrive(vx_cmd, vy_cmd, omega_cmd,
                           ax_local_total, ay_local_total, target.alpha);
    return true;
}

// â”€â”€â”€ Asymmetric kinematics + voltage safeguard (Pipeline.md Step 8) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void TrajectoryExecutor::executeAsymmetricDrive(float vx, float vy, float omega,
                                                float ax, float ay, float alpha_rot) {
    float v_bus = readBatteryVoltage();

    float target_voltages[4];
    float max_req = 0.0f;

    for (int i = 0; i < 4; ++i) {
        float v_wheel = sinf(ALPHA_WHEEL[i]) * vx
                      + cosf(ALPHA_WHEEL[i]) * vy
                      - R_CHASSIS * omega;

        float a_wheel = sinf(ALPHA_WHEEL[i]) * ax
                      + cosf(ALPHA_WHEEL[i]) * ay
                      - R_CHASSIS * alpha_rot;

        float w_motor = v_wheel / R_WHEEL;
        float a_motor = a_wheel / R_WHEEL;

        target_voltages[i] = kS * soft_sign(w_motor) + kV * w_motor + kA * a_motor;

        float abs_v = fabsf(target_voltages[i]);
        if (abs_v > max_req) max_req = abs_v;
    }

    // Proportional scaling: preserves direction ratios under voltage saturation.
    if (max_req > v_bus) {
        float scale = v_bus / max_req;
        for (int i = 0; i < 4; ++i)
            target_voltages[i] *= scale;
    }

    // Wheel-to-motor mapping (matches ALPHA_WHEEL order [FR, RR, RL, FL]):
    // TODO: verify indexâ†’motor assignment against physical wiring.
    FRMotor.setSpeed(target_voltages[0] / v_bus);
    BRMotor.setSpeed(target_voltages[1] / v_bus);
    BLMotor.setSpeed(target_voltages[2] / v_bus);
    FLMotor.setSpeed(target_voltages[3] / v_bus);
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

float TrajectoryExecutor::soft_sign(float w, float epsilon) {
    if (fabsf(w) < epsilon) return w / epsilon;
    return (w > 0.0f) ? 1.0f : -1.0f;
}

float TrajectoryExecutor::readBatteryVoltage() {
    // TODO: analogRead(BATT_ADC_PIN) * (3.3f / 1023.0f) * DIVIDER_RATIO
    return 12.0f;
}

float TrajectoryExecutor::readMouseVx() { return vxMouseMs_; }
float TrajectoryExecutor::readMouseVy() { return vyMouseMs_; }

void TrajectoryExecutor::setMouseVelocity(float vx, float vy) {
    vxMouseMs_ = vx;
    vyMouseMs_ = vy;
}

void TrajectoryExecutor::emergencyBrake() {
    FLMotor.stop();
    FRMotor.stop();
    BLMotor.stop();
    BRMotor.stop();
}
