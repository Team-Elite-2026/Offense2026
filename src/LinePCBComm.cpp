#include <LinePCBComm.h>
#include <algorithm>
#include <cmath>
#include <cstring>

LinePCBComm::LinePCBComm(HardwareSerial& serial)
    : _serial(serial) {
    memset(_payload, 0, sizeof(_payload));
    memset(_activatedVals, 0, sizeof(_activatedVals));
}

float LinePCBComm::normalize360(float angle) {
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    return angle;
}

float LinePCBComm::circularDistanceDegrees(float a, float b) {
    float diff = fabsf(normalize360(a) - normalize360(b));
    return std::min(diff, 360.0f - diff);
}

float LinePCBComm::headingDeltaDegrees(float previousHeading, float currentHeading) {
    float delta = previousHeading - currentHeading;
    while (delta < -180.0f) delta += 360.0f;
    while (delta > 180.0f) delta -= 360.0f;
    return delta;
}

void LinePCBComm::begin(uint32_t baud) {
    _serial.begin(baud);
}

uint8_t LinePCBComm::frameChecksum(uint8_t type, uint16_t len,
                                    const uint8_t* payload) {
    uint8_t cs = type ^ (uint8_t)(len & 0xFF) ^ (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) cs ^= payload[i];
    return cs;
}

void LinePCBComm::update() {
    while (_serial.available() > 0) {
        uint8_t b = (uint8_t)_serial.read();
        switch (_parseState) {

        case ParseState::MAGIC:
            if (b == LPKT_MAGIC) _parseState = ParseState::TYPE;
            break;

        case ParseState::TYPE:
            _pktType    = b;
            _parseState = ParseState::LEN_LO;
            break;

        case ParseState::LEN_LO:
            _pktLen     = b;
            _parseState = ParseState::LEN_HI;
            break;

        case ParseState::LEN_HI:
            _pktLen |= (uint16_t)b << 8;
            if (_pktLen == 0 || _pktLen > LPKT_MAX_PAYLOAD) {
                _parseState = ParseState::MAGIC;
            } else {
                _payloadIdx = 0;
                _parseState = ParseState::PAYLOAD;
            }
            break;

        case ParseState::PAYLOAD:
            _payload[_payloadIdx++] = b;
            if (_payloadIdx >= _pktLen) _parseState = ParseState::CSUM;
            break;

        case ParseState::CSUM: {
            uint8_t expected = frameChecksum(_pktType, _pktLen, _payload);
            if (b == expected) {
                onPacket(_pktType, _payload, _pktLen);
            }
            _parseState = ParseState::MAGIC;
            break;
        }
        }
    }
}

void LinePCBComm::recomputeResolvedState() {
    constexpr float kNoLineAngle = -5.0f;
    constexpr float kCrossToggleThresholdDegrees = 90.0f;

    if (_rawLineAngle == kNoLineAngle) {
        _lineAngle = kNoLineAngle;
        _avoidanceAngle = kNoLineAngle;
        _crossLine = false;
        _hasPreviousResolvedLineAngle = false;
        _hasPreviousBaseAvoidanceAngle = false;
        _previousResolvedLineAngle = kNoLineAngle;
        _previousBaseAvoidanceAngle = kNoLineAngle;
        if (_hasHeadingReference) {
            _previousRobotHeadingDegrees = _robotHeadingDegrees;
        }
        return;
    }

    bool canRotatePrevious = _hasPreviousResolvedLineAngle && _hasHeadingReference;
    float rotatedPreviousLineAngle = _previousResolvedLineAngle;
    float rotatedPreviousBaseAvoidanceAngle = _previousBaseAvoidanceAngle;
    if (canRotatePrevious) {
        float headingDelta = headingDeltaDegrees(
            _previousRobotHeadingDegrees, _robotHeadingDegrees);
        rotatedPreviousLineAngle = normalize360(
            _previousResolvedLineAngle + headingDelta);
        if (_hasPreviousBaseAvoidanceAngle) {
            rotatedPreviousBaseAvoidanceAngle = normalize360(
                _previousBaseAvoidanceAngle + headingDelta);
        }
    }

    bool lowConfidence = (_chordLength < 0.0f);

    // If the line board only has a sparse / degenerate reading, keep rotating
    // the last trusted line direction with the robot instead of snapping to a
    // single noisy angle.
    _lineAngle = _rawLineAngle;
    if (lowConfidence && canRotatePrevious) {
        _lineAngle = rotatedPreviousLineAngle;
    }

    float baseAvoidanceAngle = normalize360(_lineAngle + 180.0f);
    if (!lowConfidence && _hasPreviousBaseAvoidanceAngle) {
        float comparisonAngle = canRotatePrevious
            ? rotatedPreviousBaseAvoidanceAngle
            : _previousBaseAvoidanceAngle;
        if (circularDistanceDegrees(baseAvoidanceAngle, comparisonAngle) >
            kCrossToggleThresholdDegrees) {
            _crossLine = !_crossLine;
        }
    } else if (!_hasPreviousBaseAvoidanceAngle) {
        _crossLine = _rawCrossLine;
    }

    _avoidanceAngle = _crossLine ? _lineAngle : baseAvoidanceAngle;
    _previousResolvedLineAngle = _lineAngle;
    _hasPreviousResolvedLineAngle = true;

    if (!lowConfidence) {
        _previousBaseAvoidanceAngle = baseAvoidanceAngle;
        _hasPreviousBaseAvoidanceAngle = true;
    }

    if (_hasHeadingReference) {
        _previousRobotHeadingDegrees = _robotHeadingDegrees;
    }
}

void LinePCBComm::onPacket(uint8_t type, const uint8_t* payload, uint16_t len) {
    if (type == LPKT_DATA && len >= (uint16_t)sizeof(LinePCBDataPkt)) {
        LinePCBDataPkt p;
        memcpy(&p, payload, sizeof(p));
        _rawLineAngle      = p.lineAngle;
        _rawAvoidanceAngle = p.avoidanceAngle;
        _chordLength    = p.chordLength;
        _rawCrossLine   = (p.crossLine != 0);
        recomputeResolvedState();
    } else if (type == LPKT_DEBUG && len >= (uint16_t)sizeof(LinePCBDebugPkt)) {
        LinePCBDebugPkt p;
        memcpy(&p, payload, sizeof(p));
        memcpy(_activatedVals, p.activatedVals, sizeof(_activatedVals));
    }
}

void LinePCBComm::setDebugEnabled(bool enabled) {
    _debugEnabled = enabled;
    sendCommand(enabled ? 1u : 0u, 0u);
}

void LinePCBComm::triggerCalibration() {
    sendCommand(_debugEnabled ? 1u : 0u, 1u);
}

void LinePCBComm::endCalibration() {
    sendCommand(_debugEnabled ? 1u : 0u, 0u);
}


void LinePCBComm::sendCommand(uint8_t debugEnabled, uint8_t calibrate) {
    LinePCBCmdPkt cmd;
    cmd.debugEnabled = debugEnabled;
    cmd.calibrate    = calibrate;

    constexpr uint16_t len = (uint16_t)sizeof(LinePCBCmdPkt);
    uint8_t cksum = frameChecksum(LPKT_CMD, len, (const uint8_t*)&cmd);

    uint8_t frame[1 + 1 + 2 + len + 1];
    frame[0] = LPKT_MAGIC;
    frame[1] = LPKT_CMD;
    frame[2] = (uint8_t)(len & 0xFF);
    frame[3] = (uint8_t)(len >> 8);
    memcpy(frame + 4, &cmd, len);
    frame[4 + len] = cksum;
    _serial.write(frame, sizeof(frame));
}
