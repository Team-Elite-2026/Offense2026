#ifndef LINEPCH_COMM_H
#define LINEPCH_COMM_H

#include <Arduino.h>

// --- LinePCB <-> Main Teensy 4.1 UART protocol (Serial2, 1 Mbaud) -----------
//
// Frame: [0xA5 magic][type 1B][len_lo][len_hi][payload N B][xor_checksum 1B]
// Checksum = XOR of [type, len_lo, len_hi, payload...]
//
//   0x01  LinePCBDataPkt   LinePCB → Main  (24 B payload, every loop)
//   0x02  LinePCBDebugPkt  LinePCB → Main  (96 B payload, when debug enabled)
//   0x10  LinePCBCmdPkt    Main → LinePCB  (2 B payload, on state change)

static constexpr uint8_t  LPKT_MAGIC = 0xA5;
static constexpr uint8_t  LPKT_DATA  = 0x01;
static constexpr uint8_t  LPKT_DEBUG = 0x02;
static constexpr uint8_t  LPKT_CMD   = 0x10;

#pragma pack(push, 1)
struct LinePCBDataPkt {
    float   lineAngle;       // degrees; -5 = no line
    float   avoidanceAngle;  // degrees; -5 = no line

    float   chordLength;     // normalised [0,1]; -5 if < 2 sensors active
    uint8_t crossLine;
    uint8_t _pad[3];
};  // 24 bytes

struct LinePCBDebugPkt {
    int16_t activatedVals[48];  // 96 bytes
};

struct LinePCBCmdPkt {
    uint8_t debugEnabled;  // 1 = LinePCB should send debug packets
    uint8_t calibrate;     // 1 = LinePCB runs one calibration pass
};
#pragma pack(pop)

static constexpr uint16_t LPKT_MAX_PAYLOAD = sizeof(LinePCBDebugPkt);  // 96 bytes

class LinePCBComm {
public:
    explicit LinePCBComm(HardwareSerial& serial);
    void begin(uint32_t baud);

    // Drain Serial2 and parse incoming packets. Call every loop iteration.
    void update();

    // Feed the current robot heading so line recovery can stay stable even if
    // the robot is rotated while sitting on or beyond the boundary.
    void setRobotHeadingDegrees(float headingDegrees);

    // Send a debug-enable command to the LinePCB.
    // LinePCB will include a LinePCBDebugPkt each loop while enabled.
    void setDebugEnabled(bool enabled);

    // Send a calibration command to the LinePCB (one-shot; LinePCB auto-clears).
    void triggerCalibration();
    void endCalibration();

    float          getLineAngle()       const { return _lineAngle; }
    float          getAvoidanceAngle()  const { return _avoidanceAngle; }
    float          getChordLength()     const { return _chordLength; }
    bool           getCrossLine()       const { return _crossLine; }
    const int16_t* getActivatedVals()   const { return _activatedVals; }

private:
    HardwareSerial& _serial;

    float   _rawLineAngle      = -5.0f;
    float   _rawAvoidanceAngle = -5.0f;
    bool    _rawCrossLine      = false;

    float   _lineAngle      = -5.0f;
    float   _avoidanceAngle = -5.0f;
    float   _chordLength    = -5.0f;
    bool    _crossLine      = false;
    int16_t _activatedVals[48] = {};

    bool _debugEnabled = false;
    bool  _hasHeadingReference = false;
    float _robotHeadingDegrees = 0.0f;
    float _previousRobotHeadingDegrees = 0.0f;
    bool  _hasPreviousResolvedLineAngle = false;
    float _previousResolvedLineAngle = -5.0f;
    bool  _hasPreviousBaseAvoidanceAngle = false;
    float _previousBaseAvoidanceAngle = -5.0f;

    enum class ParseState : uint8_t { MAGIC, TYPE, LEN_LO, LEN_HI, PAYLOAD, CSUM };
    ParseState _parseState = ParseState::MAGIC;
    uint8_t    _pktType    = 0;
    uint16_t   _pktLen     = 0;
    uint16_t   _payloadIdx = 0;
    uint8_t    _payload[LPKT_MAX_PAYLOAD];

    void onPacket(uint8_t type, const uint8_t* payload, uint16_t len);
    void sendCommand(uint8_t debugEnabled, uint8_t calibrate);
    uint8_t frameChecksum(uint8_t type, uint16_t len, const uint8_t* payload);
    void recomputeResolvedState();
    static float normalize360(float angle);
    static float circularDistanceDegrees(float a, float b);
    static float headingDeltaDegrees(float previousHeading, float currentHeading);
};

#endif
