
#include <Cam.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <algorithm>
using namespace std;
Cam::Cam()
{
  ball = -5;
  yellowGoal = -5;
  blueGoal = -5;
  ballDistance = -5;
  derivative = -5;
  previousBallAngle = -5;
  previousBlueAngle = -5;
  previousYellowAngle = -5;
  previousBallDistance = -5;
  buffer = "";
}
double Cam::CamCalc()
{
  if (Serial2.available() > 0)
  {
    
    for (int i = 0; i < Serial2.available(); i++)
    {
      read = Serial2.read();
      // Serial.println(read);
      if (read == 'b')
      {
        ball = strtod(buffer.c_str(), NULL);
        ball = FilterAngle(ball, previousBallAngle);
        if ((previousBallAngle > 340 || previousBallAngle < 20) && previousBallDistance <= 25 && ball == -5)
        {
          inIntake = true;
          previousBallAngle = previousBallAngle;
          previousBallDistance = previousBallDistance;
          ball = previousBallAngle;
        }
        else
        {
          inIntake = false;
          previousBallAngle = ball;
        }
        if (switches.lightgate())
          ball = 0;
        buffer = "";
        Serial.print("ball: ");
        Serial.println(ball);
      }

      else if (read == 'c')
      {
        blueGoal = strtod(buffer.c_str(), NULL);
        blueGoal = FilterAngle(blueGoal, previousBallAngle);
        Serial.print("blue: ");
        Serial.println(blueGoal);
        buffer = "";
        previousBlueAngle = blueGoal;
      }
      else if (read == 'd')
      {
        yellowGoal = strtod(buffer.c_str(), NULL);
        yellowGoal = FilterAngle(yellowGoal, previousYellowAngle);
        Serial.print("yellow: ");
        Serial.println(yellowGoal);
        buffer = "";
        previousYellowAngle = yellowGoal;
      }
      else if (read == 'f')
      {
        sampleTime = derivativeSample;
        derivative = strtod(buffer.c_str(), NULL);
        buffer = "";
        derivativeSample = 0;
      }
      else if (read == 'a')
      {
        
        ballDistance = strtod(buffer.c_str(), NULL);
        if (inIntake == true)
        {
          Serial.println("hi");
          ballDistance = previousBallAngle;
          previousBallDistance = previousBallDistance;
        }
        if (switches.lightgate())
          ballDistance = 12;
        Serial.print("Distance: ");
        Serial.println(ballDistance);
        buffer = "";
        previousBallDistance = ballDistance;
      }
      else
      {
        buffer += read;
      }
    }
    return 0;
  }
}
void Cam::ballNotFound(int ballX, int ballY, int robotX, int robotY)
{
  if (ballX == -5 || ballY == -5)
  {
    return;
  }
  int relativeX = ballX - robotX;
  int relativeY = ballY - robotY;
  int angle = toDegrees(atan2(relativeX, relativeY));
  if (angle < 0)
    angle = 360 + angle;
  ball = angle;
  ballDistance = sqrt((pow(relativeX, 2) + pow(relativeY, 2)));
  Serial.print("Hidden Ball: ");
  Serial.println(ball);
}
double Cam::FilterAngle(double angle, double previousAngle)
{
  if (angle == -5)
  {
    return -5;
  }
  if (angle == 0)
  {
    return previousAngle;
  }
  if (angle > 360 || angle < 0)
  {
    return previousAngle;
  }
  else
  {
    return angle;
  }
}
