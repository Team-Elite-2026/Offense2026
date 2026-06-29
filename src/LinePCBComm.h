#ifndef LINEPCH_COMM_H
#define LINEPCH_COMM_H

#include <Arduino.h>

class LinePCBComm {
public:
    explicit LinePCBComm(HardwareSerial& serial);
    void begin(uint32_t baud);

    // Text stream from the LinePCB:
    // {line angle}a{avoidance angle}b{chord length}c{cross line 1/0}d
    void update();

    // This text protocol does not send active sensor debug packets.
    void setDebugEnabled(bool enabled);

    void triggerCalibration();
    void endCalibration();

    float          getLineAngle()       const { return _lineAngle; }
    float          getAvoidanceAngle()  const { return _avoidanceAngle; }
    float          getChordLength()     const { return _chordLength; }
    bool           getCrossLine()       const { return _crossLine; }
    const int16_t* getActivatedVals()   const { return _activatedVals; }

private:
    HardwareSerial& _serial;
    char   _read = '\0';
    String _buffer;
    bool   _areCalibrating = false;

    float   _lineAngle      = -5.0f;
    float   _avoidanceAngle = -5.0f;
    float   _chordLength    = -5.0f;
    bool    _crossLine      = false;
    int16_t _activatedVals[48] = {};

    void sendCommand(const String& data);
};

#endif
