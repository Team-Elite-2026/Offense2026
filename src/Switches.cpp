#include <Switches.h>
#include <Arduino.h>

Switch::Switch()
{
    pinMode(31, INPUT);
    pinMode(40, INPUT);
    pinMode(38, INPUT);
    pinMode(34, INPUT);
    pinMode(41, INPUT);
    digitalWrite(41, HIGH);
    delay(100);
}

bool Switch::start()
{
    if (digitalRead(31) == HIGH)
    {
        return true;
    }
    return false;
}
bool Switch::switchSide()
{
    if (digitalRead(38) == HIGH)
    {
        return true;
    }
    return false;
}
bool Switch::kickoff()
{
    if (digitalRead(40) == HIGH)
    {
        return true;
    }
    return false;
}
bool Switch::calibration()
{
    if (digitalRead(34) == HIGH)
    {
        return true;
    }
    return false;
}
bool Switch::lightgate()
{
    Serial.print("Lightgate: ");
    Serial.println(digitalRead(41));
    if (digitalRead(41))
    {
        return false;
    }
    else
    {
        return true;
    }
}