#include <Arduino.h>
#include <LineDetection.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>
#include <Movement.h>
#include <orbit.h>
#include <Cam.h>
#include <Defense.h>
#include <trig.h>
#include <TrajectoryExecutor.h>

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
TrajectoryExecutor trajectoryExecutor(FL, FR, BL, BR, compassSensor, switches);

enum class RobotMode
{
  Offense,
  Defense
};

RobotMode kRobotMode = RobotMode::Offense;
double defenseSpeedFactor = 0.26;
double offenseSpeedFactor = 0.22;
double lineAvoidanceSpeed = 0.15;



double lineAngle, currentOffset, orbitAngle, maxChordLength, goalAngle, avoidanceAngle;
bool aimingGoal;

void setup()
{
  if (kRobotMode == RobotMode::Offense) {
    movement.myPID->SetTunings(0.3, movement.ki, movement.kd);
  }
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
    movement.stop();
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    return;
  }

  lineDetection.Calculate();
  lineAngle  = lineDetection.getAngle();
  orbitAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);

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

  if (goalAngle == -5)
  {
    goalAngle  = 0;
    aimingGoal = false;
  }
  else
  {
    aimingGoal = true;
  }

  Serial.println("Line Angle: "        + String(lineAngle));
  Serial.println("Robot Angle: "       + String(orbitAngle));
  Serial.println("Ball Angle: "        + String(camera.ballAngle));
  Serial.println("Goal Angle: "        + String(goalAngle));
  Serial.println("Ball dist: "         + String(camera.ballDist));
  Serial.println("Zeroed angle: "      + String(compassSensor.currentOffset()));
  Serial.println("Orientation angle: " + String(compassSensor.getOrientation()));

  movement.kickBackground();

  // Always drain Serial2 so no trajectory packets are silently dropped,
  // even when line avoidance overrides movement this iteration.
  trajectoryExecutor.processSerial();

  if (!switches.start())
  {
    movement.stop();
    return;
  }

  // â”€â”€ Line avoidance â€“ highest-priority safety override â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  // The trajectory executor is paused (not called) while avoiding the line.
  // It will resume seamlessly on the next iteration once clear.
  if (lineAngle != -5)
  {
    double avoidanceAngle = lineDetection.avoidanceAngle();
    Serial.println("Avoidance angle: " + String(avoidanceAngle));
    movement.movement(avoidanceAngle, lineAvoidanceSpeed, 0, false);
    return;
  }

  // â”€â”€ Kicker: fire when ball is secure and we are aligned with the goal â”€â”€â”€â”€â”€â”€â”€
  // THIS WILL HAVE TO CHANGE
  if (switches.lightgate() && fabs(goalAngle) < 5)
  {
    movement.kick();
    return;
  }

  // â”€â”€ Trajectory execution (Pipeline.md Steps 7â€“8) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  // execute() returns false when no active chunk has been received yet,
  // or after the grace period expires â€“ fall back to orbit-based approach.
  if   (!trajectoryExecutor.execute())
  {
    if (camera.ballAngle != -5)
    {
      movement.movement(orbitAngle, offenseSpeedFactor, goalAngle, aimingGoal);
    }
    else
    {
      movement.stop();
    }
  }
}

void runDefense()
{
  if (switches.calibration())
  {
    movement.stop();
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    return;
  }

  lineDetection.Calculate();
  lineAngle = lineDetection.getAngle();
  maxChordLength = lineDetection.getChordLengthFurthestPairNormalized();
  if (lineAngle != -5)
  {
    // Updates crossLine side memory based on angle wrap jumps.
    avoidanceAngle = lineDetection.avoidanceAngle();
    Serial.println("Avoidance Angle: " + String(avoidanceAngle));
  }
  bool crossLineState = lineDetection.getCrossLine();

  double homeGoalAngle = getHomeGoalAngle();
  movement.kickBackground();

  currentOffset = compassSensor.currentOffset();

  // Serial.println("Line Angle: " + String(lineAngle));
  // Serial.println("Ball Angle: " + String(camera.ballAngle));
  // Serial.println("Home Goal Angle: " + String(homeGoalAngle));
  // Serial.println("Max Normalized Activated Sensor Distance: " + String(maxChordLength));
  Serial.println("BALL DISTANCE: " + String(camera.ballDist));
  // Serial.println("Cross Line: " + String(crossLineState ? "true" : "false"));
  // Serial.println("Current offset: " + String(currentOffset));

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
    movement.movement(camera.ballAngle, defenseSpeedFactor, 0, false);
    return;
  }
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

  // for (int i = 0; i < 10; i++) {
  //   Serial.println();
  // }

  // delay(200);

}
