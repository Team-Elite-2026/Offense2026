#include <Arduino.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>
#include <Movement.h>
#include <orbit.h>
#include <Cam.h>
#include <trig.h>
#include <LinePCBComm.h>
#include <ModeControl.h>

constexpr double pincontrolFLA = 22;
constexpr double pincontrolFLB = 23;
constexpr double pinspeedFL    = 2;
constexpr double pincontrolRLA = 18;
constexpr double pincontrolRLB = 31;
constexpr double pinspeedRL    = 4;
constexpr double pincontrolFRA = 20;
constexpr double pincontrolFRB = 21;
constexpr double pinspeedFR    = 3;
constexpr double pincontrolDribblerA = 11;
constexpr double pincontrolDribblerB = 12;
constexpr double pinspeedDribbler    = 6;
constexpr double pincontrolRRA = 9;
constexpr double pincontrolRRB = 10;
constexpr double pinspeedRR    = 5;

RobotMode kRobotMode = RobotMode::Offense;
double defenseSpeedFactor = 0.26;
double offenseSpeedFactor = 0.3;
double lineAvoidanceSpeed = 0.15;

CompassSensor compassSensor;
Switch switches;
Calibration calibration(compassSensor);
Motor FL(pincontrolFLA, pincontrolFLB, pinspeedFL);
Motor FR(pincontrolFRA, pincontrolFRB, pinspeedFR);
Motor BL(pincontrolRLA, pincontrolRLB, pinspeedRL);
Motor BR(pincontrolRRA, pincontrolRRB, pinspeedRR);
Movement movement(FL, FR, BL, BR, compassSensor);
Orbit orbit(1);
Cam camera;
LinePCBComm linePCBComm(Serial2);
ModeControl modeControl(Serial8, linePCBComm, compassSensor, movement, kRobotMode);

double lineAngle, currentOffset, orbitAngle, maxChordLength, goalAngle, avoidanceAngle;
bool aimingGoal;

void setup()
{
  // if (kRobotMode == RobotMode::Offense) {
  //   movement.myPID->SetTunings(0.3, movement.ki, movement.kd);
  // }
  // Serial.begin(9600);
  // Serial.println("Testing Run");
  // Serial3.begin(2000000);
  // compassSensor.begin();
  // compassSensor.callibrate();
  // modeControl.begin(115200);
  // modeControl.applyRobotModeSettings();
  // linePCBComm.begin(1000000);
  pinMode(30, OUTPUT);
  digitalWrite(30, LOW);
  // delay(100);
  // digitalWrite(30, LOW);


}

double getHomeGoalAngle()
{
  if (switches.goalSide())
  {
    return camera.yellowGoal;
  }
  return camera.blueGoal;
}

void runOffense()
{
  if (modeControl.state.lineCalibrationActive)
  {
    movement.stop();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
  }
  else
  {
    // Serial.println("Testing Line Sensors");
    linePCBComm.update();
    camera.CamCalc();
    lineAngle = linePCBComm.getLineAngle();

    orbitAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);
    if (modeControl.state.goalIsBlue)
    {
      Serial.println("blue goal");
      goalAngle = camera.blueGoal;
    }
    else
    {
      Serial.println("yellow goal");
      goalAngle = camera.yellowGoal;
    }

    if (goalAngle == -5)
    {
      goalAngle = 0;
      aimingGoal = false;
    }
    else
    {
      aimingGoal = true;
    }

    // Serial.println("Offset: " + String(compassSensor.currentOffset()));
    Serial.println("Line Angle: " + String(lineAngle));
    Serial.println("Robot Angle: " + String(orbitAngle));
    Serial.println("Ball Angle: " + String(camera.ballAngle));
    Serial.println("Goal Angle: " + String(goalAngle));
    Serial.println("Ball dist:" + String(camera.ballDist));
    Serial.println("Robot Heading: " + String(compassSensor.currentOffset()));
    // Serial.println("Orientation angle" + String(compassSensor.getOrientation()));
    movement.kickBackground();
    if (lineAngle == -5)
    {
      if (modeControl.isStartEnabled())
      {
        if (modeControl.doWeHaveBall())
        {
          // movement.movement(0, 0.2, goalDesiredFieldAngle, aimingGoal); 
          if (fabs(goalAngle) < 5)
          { // if close to goal angle, kick
            Serial.println("KICKKKKKKKKKKKKKKK");
            Serial.println();
            movement.kick(); // wanna kick the ball to the goal
          }
        }
        else if (camera.ballAngle != -5)
        {
          // movement.movement(orbitAngle, offenseSpeedFactor, 0, false); // wanna try to face dir of ball to get into dribbler so no trying to aim to the goal
          movement.movement(orbitAngle, offenseSpeedFactor, goalAngle, aimingGoal); // j using default orbit aiming towards the goal if seen
        }
        else
        {
          movement.stop();
        }
      }
      else
      {
        movement.stop();

      }
    }
    else
    {
      double avoidanceAngle = linePCBComm.getAvoidanceAngle();
      Serial.println("Avoidance angle: " + String(avoidanceAngle));
      if (modeControl.isStartEnabled())
      {
        movement.movement(avoidanceAngle, lineAvoidanceSpeed, 0 , false); // Not turning while avoiding line can cause extra rotation when goal scoring meaning we still want to correct when we're goal scoring
      }
      else
      {
        movement.stop();
      }
    }
  }
}

// void runDefense()
// {
//   if (switches.calibration())
//   {
//     movement.stop();
//     calibration.calibrateLineSensors();
//     calibration.calibrateCompassSensor();
//     Serial.println("Calibrating");
//     return;
//   }

//   lineDetection.Calculate();
//   camera.CamCalc();
//   lineAngle = lineDetection.getAngle();
//   maxChordLength = lineDetection.getChordLengthFurthestPairNormalized();
//   if (lineAngle != -5)
//   {
//     // Updates crossLine side memory based on angle wrap jumps.
//     avoidanceAngle = lineDetection.avoidanceAngle();
//     Serial.println("Avoidance Angle: " + String(avoidanceAngle));
//   }
//   bool crossLineState = lineDetection.getCrossLine();

//   double homeGoalAngle = getHomeGoalAngle();
//   movement.kickBackground();

//   currentOffset = compassSensor.currentOffset();

//   // Serial.println("Line Angle: " + String(lineAngle));
//   // Serial.println("Ball Angle: " + String(camera.ballAngle));
//   // Serial.println("Home Goal Angle: " + String(homeGoalAngle));
//   // Serial.println("Max Normalized Activated Sensor Distance: " + String(maxChordLength));
//   Serial.println("BALL DISTANCE: " + String(camera.ballDist));
//   // Serial.println("Cross Line: " + String(crossLineState ? "true" : "false"));
//   // Serial.println("Current offset: " + String(currentOffset));

//   if (!switches.start())
//   {
//     movement.stop();
//     return;
//   }

//   if (camera.ballAngle == -5)
//   {
//     movement.stop();
//     return;
//   }

//   if (homeGoalAngle == -5)
//   {
//     movement.movement(camera.ballAngle, defenseSpeedFactor, 0, false);
//     return;
//   }
// }


void loop()
{
  linePCBComm.update();
  linePCBComm.setRobotHeadingDegrees(compassSensor.currentOffset());
  modeControl.readCommands();

  if (kRobotMode == RobotMode::Offense)
  {
    runOffense();
  }
  else
  {
    // runDefense();
  }
  modeControl.sendTelemetry(lineAngle, avoidanceAngle);
}
