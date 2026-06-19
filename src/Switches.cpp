#include <Switches.h>
#include <Arduino.h>

Switch::Switch()
{
    pinMode(kStartPin,     INPUT);
    pinMode(kGoalSidePin,  INPUT);
    pinMode(kLightGatePin, INPUT);
    digitalWrite(kLightGatePin, HIGH);
    delay(100);
}

bool Switch::start()
{
    return digitalRead(kStartPin) == HIGH;
}

bool Switch::goalSide()
{
    return digitalRead(kGoalSidePin) == HIGH;
}

bool Switch::lightgate()
{
    return !digitalRead(kLightGatePin);
}