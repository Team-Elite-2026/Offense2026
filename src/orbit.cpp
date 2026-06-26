#include <orbit.h>
#include <math.h>
#include <trig.h>
Orbit::Orbit(int robotNum)
{
    physicalRobot = robotNum;
}

double Orbit::CalculateRobotAngle(double ballAngle, double distance)
{
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

    double newballAngle = ballAngle > 180 ? (360 - ballAngle) : ballAngle;
    double orbitValue;


    if(physicalRobot == 1){ // Offense
        orbitValue = Trig::min(90, 2*M_PI * exp(0.05 * newballAngle));
    }
    else{ 
        orbitValue = Trig::min(90, 8 * exp(0.033 * newballAngle));
    }

    double outputSum = orbitValue * dampenVal;

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
    // Serial.print("robot Angle: ");
    // Serial.println(robotAngle);
    return robotAngle;
}

double Orbit::GetToPosition(int targetX, int targetY, int currentX, int currentY)
{

    int relativex = -1;
    int relativey = -1;
    relativex = targetX - currentX;
    relativey = targetY - currentY;
    double distance = sqrt(pow(relativex, 2) + pow(relativey, 2));
    if (distance <= 15)
    {
        return -1;
    }
    homeAngle = (atan2(relativex, relativey)) * 180/M_PI;
    if (homeAngle < 0)
    {
        homeAngle = homeAngle + 360;
    }
    // Serial.print("home Angle: ");
    // Serial.println(homeAngle);
    return homeAngle;
}
