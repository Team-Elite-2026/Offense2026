#include <LinePCBComm.h>
#include <cstring>

LinePCBComm::LinePCBComm(HardwareSerial& serial)
    : _serial(serial) {
    memset(_payload, 0, sizeof(_payload));
    memset(_activatedVals, 0, sizeof(_activatedVals));
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

void LinePCBComm::onPacket(uint8_t type, const uint8_t* payload, uint16_t len) {
    if (type == LPKT_DATA && len >= (uint16_t)sizeof(LinePCBDataPkt)) {
        LinePCBDataPkt p;
        memcpy(&p, payload, sizeof(p));
        _lineAngle      = p.lineAngle;
        _avoidanceAngle = p.avoidanceAngle;
        _mouseVx        = p.mouseVx;
        _mouseVy        = p.mouseVy;
        _chordLength    = p.chordLength;
        _crossLine      = (p.crossLine != 0);
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
