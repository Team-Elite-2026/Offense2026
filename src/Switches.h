
#ifndef SWITCH_H
#define SWITCH_H

class Switch
{
public:
    static constexpr int kStartPin     = 38;
    static constexpr int kGoalSidePin  = 31;
    static constexpr int kLightGatePin = 41;

    Switch();
    bool start();
    bool goalSide();
    bool lightgate();
};
#endif