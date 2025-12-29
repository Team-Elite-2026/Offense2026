
#include <Cam.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <algorithm>
using namespace std;
Cam::Cam()
{
  ballAngle = -5;
  yellowGoal = -5;
  blueGoal = -5;
  ballDist = -5;
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
        ballAngle = strtod(buffer.c_str(), NULL);
        buffer = "";
        Serial.print("ball angle: ");
        Serial.println(ballAngle);
      }
      else if (read == 'a') {
        ballDist = strtod(buffer.c_str(), NULL);
        if (switches.lightgate())
          ballDist = 12;
        buffer = "";
        Serial.print("ball dist: ");
        Serial.println(ballDist);
      }
      else if (read == 'c')
      {
        blueGoal = strtod(buffer.c_str(), NULL);
        buffer = "";
        Serial.print("blue: ");
        Serial.println(blueGoal);
      }
      else if (read == 'd')
      {
        yellowGoal = strtod(buffer.c_str(), NULL);
        Serial.print("yellow: ");
        Serial.println(yellowGoal);
        buffer = "";
      }
      else
      {
        buffer += read;
      }
    }
    return 0;
  }
}