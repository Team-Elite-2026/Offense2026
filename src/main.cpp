#include <Arduino.h>
#include <LineDetection.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>
#include <Movement.h>
#include <orbit.h>
#include <Cam.h>
#include <Defense.h>

double pincontrolRLA = 22;
double pincontrolRLB = 23;
double pinspeedRL = 2;
double pincontrolRRA = 20;
double pincontrolRRB = 21;
double pinspeedRR = 3;
double pincontrolFRA = 18;
double pincontrolFRB = 19;
double pinspeedFR = 4;
double pincontrolFLA = 9;
double pincontrolFLB = 10;
double pinspeedFL = 5;

LineDetection lineDetection;
CompassSensor compassSensor;
Switch switches;
Calibration calibration(lineDetection, compassSensor);
Motor FL(pincontrolFLA, pincontrolFLB, pinspeedFL);
Motor FR(pincontrolFRA, pincontrolFRB, pinspeedFR);
Motor BL(pincontrolRLA, pincontrolRLB, pinspeedRL);
Motor BR(pincontrolRRA, pincontrolRRB, pinspeedRR);
Movement movement(FL, FR, BL, BR, compassSensor);
Orbit orbit(1);
Cam camera;
Defense defense;

enum class RobotMode
{
  Offense,
  Defense
};

constexpr RobotMode kRobotMode = RobotMode::Defense;
constexpr double kDefenseSpeed = 0.2;

double lineAngle;
double robotAngle;
double goalAngle;
double goalDesiredFieldAngle;
bool aimingGoal;

void setup()
{
  Serial.begin(9600);
  Serial2.begin(2000000);
  compassSensor.begin();
  compassSensor.callibrate();
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
  if (switches.calibration())
  {
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
  }
  else
  {
    // Serial.println("Testing Line Sensors");
    lineDetection.Calculate();
    camera.CamCalc();
    lineAngle = lineDetection.getAngle();
    robotAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);
    if (switches.goalSide())
    {
      Serial.println("blue goal");
      goalAngle = camera.blueGoal;
    }
    else
    {
      Serial.println("yellow goal");
      goalAngle = camera.yellowGoal;
    }

    // Camera goal angles are robot-relative. Convert to field-relative for heading PID.
    if (goalAngle == -5)
    {
      goalDesiredFieldAngle = 0;
      aimingGoal = false;
    }
    else
    {
      goalDesiredFieldAngle = goalAngle;
      aimingGoal = true;
    }

    // Serial.println("Offset: " + String(compassSensor.currentOffset()));
    Serial.println("Line Angle: " + String(lineAngle));
    Serial.println("Robot Angle: " + String(robotAngle));
    Serial.println("Ball Angle: " + String(camera.ballAngle));
    Serial.println("Goal Angle: " + String(goalAngle));
    movement.kickBackground();
    if (lineAngle == -5)
    {
      if (switches.start())
      {
        if (switches.lightgate())
        {
          movement.movement(0, 0.2, goalDesiredFieldAngle, aimingGoal); // wanna kick the ball to the goal
          if (abs(compassSensor.currentOffset() - goalDesiredFieldAngle) < 5)
          { // if close to goal angle, kick
            movement.kick();
          }
          movement.kick();
        }
        else if (camera.ballAngle != -5)
        {
          movement.movement(robotAngle, 0.2, goalDesiredFieldAngle, aimingGoal); // wanna try to face ball to get into dribbler
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
      double avoidanceAngle = lineDetection.avoidanceAngle();
      Serial.println("Avoidance angle: " + String(avoidanceAngle));
      if (switches.start())
      {
        movement.movement(avoidanceAngle, 0.2, 0, aimingGoal); // Not turning while avoiding line can cause extra rotation when goal scoring meaning we still want to correct when we're goal scoring
      }
      else
      {
        movement.stop();
      }
    }
  }
}

void runDefense()
{
  if (switches.calibration())
  {
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    return;
  }

  lineDetection.Calculate();
  camera.CamCalc();
  lineAngle = lineDetection.getAngle();

  double homeGoalAngle = getHomeGoalAngle();
  movement.kickBackground();

  Serial.println("Line Angle: " + String(lineAngle));
  Serial.println("Ball Angle: " + String(camera.ballAngle));
  Serial.println("Home Goal Angle: " + String(homeGoalAngle));

  if (lineAngle != -5 && lineDetection.getCordLength() > 0.3)
  {
    double avoidance = lineDetection.avoidanceAngle();
    Serial.println("Defense Avoidance Angle: " + String(avoidance));
    if (switches.start())
    {
      movement.movement(avoidance, kDefenseSpeed, 0, false);
    }
    else
    {
      movement.stop();
    }
    return;
  }

  if (!switches.start())
  {
    movement.stop();
    return;
  }

  if (camera.ballAngle == -5)
  {
    movement.stop();
    return;
  }

  if (homeGoalAngle == -5)
  {
    movement.movement(camera.ballAngle, kDefenseSpeed, 0, false);
    return;
  }

  double defenseMoveAngle = defense.defenseCalc(
      camera.ballAngle,
      homeGoalAngle,
      compassSensor.currentOffset());

  if (defenseMoveAngle < 0)
  {
    movement.stop();
    return;
  }

  movement.movement(defenseMoveAngle, kDefenseSpeed, 0, false);
}

void loop()
{
  if (kRobotMode == RobotMode::Offense)
  {
    runOffense();
  }
  else
  {
    runDefense();
  }
}
