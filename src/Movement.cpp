#include <Movement.h>
#include <math.h>
#include <cmath>
#include <cstdlib>
#include <trig.h>

namespace
{
  /** Shortest turn error: current - goal, wrapped to [-180, 180] */
  double fieldHeadingError(CompassSensor& compass, double fieldGoal)
  {
    return Trig::wrapAngle((double)compass.currentOffset() - fieldGoal);
  }
}

Movement::Movement(Motor& FLMotor, Motor& FRMotor, Motor& BLMotor, Motor& BRMotor, CompassSensor& compassSensor)
    : FLMotor(FLMotor), FRMotor(FRMotor), BLMotor(BLMotor), BRMotor(BRMotor), compassSensor(compassSensor)
{
    myPID = new PID(&Input, &Output, &Setpoint, kp, ki, kd, REVERSE);
    myPID->SetMode(AUTOMATIC);

    myPID->SetOutputLimits(0, 100);
    myPID->SetSampleTime(2);
    pinMode(kickerPin, OUTPUT);
}

double Movement::findCorrection(double goalDirection) {
  double correction = 0;
  double orientationDiff = compassSensor.currentOffset() - goalDirection;
  
  Serial.println("Orientation Diff For Zero: " + String(orientationDiff));

  Input = abs(orientationDiff);
  myPID->Compute();

  if (abs(orientationDiff) < 5) {
    correction = 0;
  } if (orientationDiff > 90) {
    correction = -1;
  } else if (orientationDiff < -90) {
    correction = 1;
  } else if (orientationDiff > 0) {
    correction = -1 * (Output / 100);
  } else if (orientationDiff < 0) {
    correction = (Output / 100);
  }

  // Serial.println("Correction: " + String(correction));

  return correction;
}
double Movement::findCorrectionForGoal(double goalDirection) { // Makes the oritentation Diff the goal angle bc goal angle is already relative to the robot direction
  double correction = 0;
  double orientationDiff = goalDirection;
  
  Serial.println("Orientation Diff For Goal: " + String(orientationDiff));

  Input = abs(orientationDiff);
  myPID->Compute();

  if (abs(orientationDiff) < 5) {
    correction = 0;
  } if (orientationDiff > 90) {
    correction = -1;
  } else if (orientationDiff < -90) {
    correction = 1;
  } else if (orientationDiff > 0) {
    correction = -1 * (Output / 100);
  } else if (orientationDiff < 0) {
    correction = (Output / 100);
  }

  // Serial.println("Correction: " + String(correction));

  return correction;
}

// Need to add orientation to the movement function
void Movement::movement(double intended_movement_angle, double speedfactor, double desiredOrientation, bool AimingGoal) {
  intended_movement_angle -= 180;

  if (intended_movement_angle < 0) {
      intended_movement_angle += 360;
  }

  double powerFR = Trig::Sin(intended_movement_angle - 55);
  double powerRR = Trig::Sin(intended_movement_angle - 125);
  double powerRL = Trig::Sin(intended_movement_angle - 235);
  double powerFL = Trig::Sin(intended_movement_angle - 305);

  max_power = fmax(fmax(abs(powerFR), abs(powerFL)), fmax(abs(powerRR), abs(powerRL)));

  powerFR = powerFR / max_power;
  powerFL = powerFL / max_power;
  powerRR = powerRR / max_power;
  powerRL = powerRL / max_power;

  Serial.println("Before Finding Correction");
  double correction;
  if(!AimingGoal)
    correction = findCorrection(desiredOrientation);
  else
    correction = -1 * findCorrectionForGoal(desiredOrientation);

  powerFR -= correction;
  powerFL -= correction;
  powerRR -= correction;
  powerRL -= correction;
  
  max_power = fmax(fmax(abs(powerFR), abs(powerFL)), fmax(abs(powerRR), abs(powerRL)));

  powerFR = powerFR / max_power;
  powerFL = powerFL / max_power;
  powerRR = powerRR / max_power;
  powerRL = powerRL / max_power;



  if (powerFL > 1) {
    powerFL = 1;
  } else if (powerFL < -1) {
    powerFL = -1;
  }

  if (powerFR > 1) {
    powerFR = 1;
  } else if (powerFR < -1) {
    powerFR = -1;
  }

  if (powerRL > 1) {
    powerRL = 1;
  } else if (powerRL < -1) {
    powerRL = -1;
  }

  if (powerRR > 1) {
    powerRR = 1;
  } else if (powerRR < -1) {
    powerRR = -1;
  }


  this->FLMotor.setSpeed(speedfactor * powerFL);
  this->FRMotor.setSpeed(speedfactor * powerFR);
  this->BLMotor.setSpeed(speedfactor * powerRL);
  this->BRMotor.setSpeed(speedfactor * powerRR);
}

/** Same PID + piecewise logic as findCorrection, but for in-place spin (no Serial). */
static double fieldHeadingPidSpin(CompassSensor& compass, PID* myPID, double& input, double& output, double fieldGoal)
{
  double err = fieldHeadingError(compass, fieldGoal);
  input = std::fabs(err);
  myPID->Compute();
  if (std::fabs(err) < 5) {
    return 0.0;
  }
  if (err > 90) {
    return -1.0;
  } else if (err < -90) {
    return 1.0;
  } else if (err > 0) {
    return -1.0 * (output / 100.0);
  } else {
    return (output / 100.0);
  }
}

void Movement::rotateToFieldHeading(double fieldHeadingDeg, double speedFactor)
{
  double spin = fieldHeadingPidSpin(compassSensor, myPID, Input, Output, fieldHeadingDeg);
  if (std::fabs(spin) < 1e-6) {
    stop();
    return;
  }
  // Mecanum in-place spin (tune sign if robot turns the wrong way)
  double s = speedFactor;
  FLMotor.setSpeed(s * (-spin));
  FRMotor.setSpeed(s * spin);
  BLMotor.setSpeed(s * (-spin));
  BRMotor.setSpeed(s * spin);
}

void Movement::rotateByRobotRelative(double relativeDeltaDeg, double speedFactor)
{
  double target = Trig::wrapAngle(compassSensor.currentOffset() + relativeDeltaDeg);
  rotateToFieldHeading(target, speedFactor);
}

void Movement::rotateToGoal(double goalDirection, double speedFactor)
{
  // Match findCorrectionForGoal + the negation used in movement(..., AimingGoal true)
  double err = Trig::wrapAngle(goalDirection);
  Input = std::fabs(err);
  myPID->Compute();
  double c = 0.0;
  if (std::fabs(err) < 5) {
    stop();
    return;
  }
  if (err > 90) {
    c = -1.0;
  } else if (err < -90) {
    c = 1.0;
  } else if (err > 0) {
    c = -1.0 * (Output / 100.0);
  } else {
    c = (Output / 100.0);
  }
  double spin = -c;
  double s = speedFactor;
  FLMotor.setSpeed(s * (-spin));
  FRMotor.setSpeed(s * spin);
  BLMotor.setSpeed(s * (-spin));
  BRMotor.setSpeed(s * spin);
}

void Movement::circle() {
    this->FLMotor.setSpeed(0.2);
    this->FRMotor.setSpeed(0.2);
    this->BLMotor.setSpeed(0.2);
    this->BRMotor.setSpeed(0.2);
}

void Movement::kick() {
    if (timer > (kickHold + 2000))
    {
        timer = 0;
    }
    if (timer <= kickHold)
    {
        digitalWrite(kickerPin, HIGH);
    }
    else
    {
        digitalWrite(kickerPin, LOW);
    }
    active = 0;
}
void Movement::kickBackground()
{
    if (active > 2 && timer > kickHold)
    {
        digitalWrite(kickerPin, LOW);
    }
}

void Movement::stop() {
    this->FLMotor.setSpeed(0);
    this->FRMotor.setSpeed(0);
    this->BLMotor.setSpeed(0);
    this->BRMotor.setSpeed(0);
}
