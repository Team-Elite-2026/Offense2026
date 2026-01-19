
#ifndef SWITCH_H
#define SWITCH_H

class Switch
{
public:
    Switch();
    bool start();
    bool goalSide();
    bool kickoff();
    bool calibration();
    bool lightgate();

private:
    int recieverPin = 41;
};
#endif