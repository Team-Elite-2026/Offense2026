#include <LinePCBComm.h>
#include <stdlib.h>
#include <string.h>

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

        if (_read == 'a') {
            _lineAngle = strtod(_buffer.c_str(), NULL);
            _buffer = "";
        } else if (_read == 'b') {
            _avoidanceAngle = strtod(_buffer.c_str(), NULL);
            _buffer = "";
        } else if (_read == 'c') {
            _chordLength = strtod(_buffer.c_str(), NULL);
            _buffer = "";
        } else if (_read == 'd') {
            _crossLine = strtod(_buffer.c_str(), NULL) == 1.0;
            _buffer = "";
        } else if (_read == '\n' || _read == '\r') {
            _buffer = "";
        } else {
            _buffer += _read;
        }
    }
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
