#include <orbit.h>
#include <math.h>
#include <trig.h>
#include <Arduino.h>

Orbit::Orbit(int robotNum)
{
    physicalRobot = robotNum;
    kd = 0.3;
}

double Orbit::CalculateRobotAngle(double ballAngle, double distance, double derivative, int sampleTime)
{
    double dTerm = 0;
    if (derivative != -5 && sampleTime > 0)
    {
        double sampleTimeInSec = static_cast<double>(sampleTime) / 1000.0;
        dTerm = kd * (derivative / sampleTimeInSec);
    }

    distance = distance / 150;
    if (distance > 1)
    {
        distance = 1;
    }
    distance = 1 - distance;
    // Serial.print("calculated distance: ");
    // Serial.println(distance);
    // double dampenVal = min(1, 0.025 * exp(4.5 * distance));
    double dampenVal = Trig::min(1, 0.02 * exp(4.5 * distance));
    // Serial.print("dampen val: ");
    // Serial.println(dampenVal);

    // takes absolute value of ball angle from 0 - 180 range
    double newballAngle = ballAngle > 180 ? (360 - ballAngle) : ballAngle;
    double orbitValue;

    orbitValue = Trig::min(90, 4 * exp(0.1 * (newballAngle-30)));

    double outputSum = orbitValue * dampenVal;
    if (dTerm > 3)
    {
        outputSum -= dTerm;
    }

    // Serial.print("Orbit val before: ");
    // Serial.println(orbitvalue);
    // orbitvalue = orbitvalue * dampenVal;
    // Serial.print("Orbit val after: ");
    // Serial.println(orbitvalue);
    robotAngle = ballAngle + (ballAngle > 180 ? -1 : 1) * (outputSum);
    if (robotAngle >= 360)
    {
        robotAngle -= 360;
    }
    else if (robotAngle < 0)
    {
        robotAngle += 360;
    }
    Serial.print("robot Angle: ");
    Serial.println(robotAngle);
    return robotAngle;
}
