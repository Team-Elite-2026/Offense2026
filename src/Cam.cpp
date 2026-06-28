
#include <Cam.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <ModeControl.h>
#include <Movement.h>
using namespace std;
Cam::Cam()
{
  ballAngle = -5;
  yellowGoal = -5;
  blueGoal = -5;
  ballDist = -5;
  derivative = -5;
  sampleTime = 0;
  buffer = "";
  movement = NULL;
  modeControl = NULL;
  poseX = -5;
  poseY = -5;
}

void Cam::setMovement(Movement* movement)
{
  this->movement = movement;
}

void Cam::setModeControl(ModeControl* modeControl)
{
  this->modeControl = modeControl;
}

double Cam::CamCalc()
{
  if (Serial3.available() > 0)
  {
    
    for (int i = 0; i < Serial3.available(); i++)
    {
      read = Serial3.read();
      // Serial.println(read);
      if (read == 'b')
      {
        ballAngle = strtod(buffer.c_str(), NULL);
        buffer = "";
        // Serial.print("ball angle: ");
        // Serial.println(ballAngle);
      }
      else if (read == 'a') {
        ballDist = strtod(buffer.c_str(), NULL);
        if (modeControl != NULL && modeControl->doWeHaveBall())
          ballDist = 12;
        buffer = "";
        // Serial.print("ball dist: ");
        // Serial.println(ballDist);
      }
      else if (read == 'c')
      {
        blueGoal = strtod(buffer.c_str(), NULL);
        if (blueGoal > 180) {
          blueGoal -= 360;
        }
        buffer = "";
        // Serial.print("blue: ");
        // Serial.println(blueGoal);
      }
      else if (read == 'd')
      {
        yellowGoal = strtod(buffer.c_str(), NULL);
        if (yellowGoal > 180) {
          yellowGoal -= 360;
        }
        // Serial.print("yellow: ");
        // Serial.println(yellowGoal);
        buffer = "";
      }
      else if (read == 'f')
      {
        sampleTime = derivativeSample;
        derivative = strtod(buffer.c_str(), NULL);
        buffer = "";
        derivativeSample = 0;
      }
      else if (read == 'x')
      {
        poseX = strtod(buffer.c_str(), NULL);
        // Serial.print("poseX: ");
        // Serial.println(poseX);
        buffer = "";
      }
      else if (read == 'y')
      {
        poseY = strtod(buffer.c_str(), NULL);
        // Serial.print("poseY: ");
        // Serial.println(poseY);
        if (movement != NULL && poseX != -5 && poseY != -5)
        {
          movement->currentPose.x = poseX * 10;
          movement->currentPose.y = poseY * 10;
          // Serial.print("currentPose x: ");
          // Serial.println(movement->currentPose.x);
          // Serial.print("currentPose y: ");
          // Serial.println(movement->currentPose.y);
        }
        buffer = "";
      }
      else if (read == '\n' || read == '\r')
      {
        buffer = "";
      }
      else
      {
        buffer += read;
      }
    }
    return 0;
  }
  return 0;
}
