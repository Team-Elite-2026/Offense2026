#include <LinePCBComm.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr unsigned int kMaxPacketFieldLength = 16;
}

LinePCBComm::LinePCBComm(HardwareSerial& serial)
    : _serial(serial) {
    memset(_activatedVals, 0, sizeof(_activatedVals));
}

void LinePCBComm::begin(uint32_t baud) {
    _serial.begin(baud);
}

void LinePCBComm::update() {
    sendCommand(_areCalibrating ? "1" : "0");

    while (_serial.available() > 0) {
        _read = (char)_serial.read();
        processIncomingByte(_read);
    }
}

void LinePCBComm::processIncomingByte(char incoming) {
    if (incoming == '\n' || incoming == '\r') {
        resetPacketParser();
        return;
    }

    if (incoming == 'a' || incoming == 'b' || incoming == 'c' || incoming == 'd') {
        float parsedValue = 0.0f;
        if (!parseBufferedNumber(parsedValue)) {
            resetPacketParser();
            return;
        }

        if (incoming == 'a' && _packetField == PacketField::LineAngle) {
            _pendingLineAngle = parsedValue;
            _packetField = PacketField::AvoidanceAngle;
            _buffer = "";
            return;
        }

        if (incoming == 'b' && _packetField == PacketField::AvoidanceAngle) {
            _pendingAvoidanceAngle = parsedValue;
            _packetField = PacketField::ChordLength;
            _buffer = "";
            return;
        }

        if (incoming == 'c' && _packetField == PacketField::ChordLength) {
            _pendingChordLength = parsedValue;
            _packetField = PacketField::CrossLine;
            _buffer = "";
            return;
        }

        if (incoming == 'd' && _packetField == PacketField::CrossLine) {
            _pendingCrossLine = parsedValue == 1.0f;
            commitPendingPacket();
            resetPacketParser();
            return;
        }

        resetPacketParser();
        return;
    }

    if (_buffer.length() >= kMaxPacketFieldLength) {
        resetPacketParser();
        return;
    }

    _buffer += incoming;
}

bool LinePCBComm::parseBufferedNumber(float& value) const {
    if (_buffer.length() == 0) {
        return false;
    }

    char* end = nullptr;
    const char* raw = _buffer.c_str();
    double parsed = strtod(raw, &end);
    if (end == raw || *end != '\0') {
        return false;
    }

    value = (float)parsed;
    return true;
}

void LinePCBComm::commitPendingPacket() {
    _lineAngle = _pendingLineAngle;
    _avoidanceAngle = _pendingAvoidanceAngle;
    _chordLength = _pendingChordLength;
    _crossLine = _pendingCrossLine;
}

void LinePCBComm::resetPacketParser() {
    _buffer = "";
    _packetField = PacketField::LineAngle;
}

void LinePCBComm::setDebugEnabled(bool enabled) {
    (void)enabled;
}

void LinePCBComm::triggerCalibration() {
    _areCalibrating = true;
}

void LinePCBComm::endCalibration() {
    _areCalibrating = false;
}

void LinePCBComm::sendCommand(const String& data) {
    _serial.print(data);
}
