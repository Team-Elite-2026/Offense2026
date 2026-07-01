#include <Movement.h>
#include <math.h>
#include <RobotConfig.h>
#include <trig.h>

namespace {
constexpr double kPoseArrivalToleranceMm  = 20.0;
constexpr double kPoseFullSpeedDistanceMm = 250.0;
constexpr double kPoseMinSpeedFactor      = 0.06;
constexpr double kPoseMaxSpeedFactor      = 0.12;
}

Movement::Movement(Motor& FLMotor, Motor& FRMotor, Motor& BLMotor, Motor& BRMotor, Motor& dribblerMotor, CompassSensor& compassSensor)
    : FLMotor(FLMotor), FRMotor(FRMotor), BLMotor(BLMotor), BRMotor(BRMotor), dribblerMotor(dribblerMotor), compassSensor(compassSensor)
{
    myPID = new PID(&Input, &Output, &Setpoint, kp, ki, kd, REVERSE);
    myPID->SetMode(AUTOMATIC);

    myPID->SetOutputLimits(0, 100);
    myPID->SetSampleTime(2);
}

double Movement::findCorrectionRelZero(double goalDirection) {
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

  Serial.println("Correction: " + String(correction));

  return correction;
}
double Movement::findCorrectionRelOffset(double goalDirection) { // Makes the oritentation Diff the goal angle bc goal angle is already relative to the robot direction
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
    correction = findCorrectionRelZero(desiredOrientation);
  else
    correction = -1 * findCorrectionRelOffset(desiredOrientation);

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

  if (robotDebugNoMoveMode) {
    Serial.println("DEBUG NO MOVE: drive motor command suppressed");
    this->stop();
    return;
  }

  this->FLMotor.setSpeed(speedfactor * powerFL);
  this->FRMotor.setSpeed(speedfactor * powerFR);
  this->BLMotor.setSpeed(speedfactor * powerRL);
  this->BRMotor.setSpeed(speedfactor * powerRR);
}

void Movement::circle() {
    this->FLMotor.setSpeed(0.2);
    this->FRMotor.setSpeed(0.2);
    this->BLMotor.setSpeed(0.2);
    this->BRMotor.setSpeed(0.2);
}

// Spins the robot in place. Negative spinSpeed spins left, positive spins right.
void Movement::spin(double spinSpeed) {
    this->FLMotor.setSpeed(spinSpeed);
    this->FRMotor.setSpeed(spinSpeed);
    this->BLMotor.setSpeed(spinSpeed);
    this->BRMotor.setSpeed(spinSpeed);
}

// Sets the dribbler motor. speedFactor is a normalized factor in [-1, 1] (PWM / 255).
void Movement::setDribbler(double speedFactor) {
    this->dribblerMotor.setSpeed(speedFactor);
}

void Movement::kick() {
    if (timer > (kickHold + 1000))
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

void Movement::runAll() {
    this->FLMotor.setSpeed(0.2);
    this->FRMotor.setSpeed(0.2);
    this->BLMotor.setSpeed(0.2);
    this->BRMotor.setSpeed(0.2);

    delay(2000);

    this->FLMotor.setSpeed(-0.2);
    this->FRMotor.setSpeed(-0.2);
    this->BLMotor.setSpeed(-0.2);
    this->BRMotor.setSpeed(-0.2);

    delay(2000);

}


void Movement::PlanToPose(Point desiredPose) {
  this->currentPose.heading = compassSensor.currentOffset();
  double speedfactor = computeSpeedFactor(this->currentPose, desiredPose);
  if (speedfactor <= 0) {
    this->stop();
    return;
  }

  double movementAngle = Trig::getAngle(this->currentPose, desiredPose);
  Serial.print("Movement angle: ");
  Serial.println(movementAngle);
  this->movement(movementAngle, speedfactor, desiredPose.heading, false);

}

double Movement::computeSpeedFactor(Point currentPose, Point desiredPose) {
  double dist = Trig::getDist(currentPose, desiredPose);
  Serial.print("Current pose: ");
  Serial.print(currentPose.x);
  Serial.print(", ");
  Serial.println(currentPose.y);
  Serial.print("Desired pose: ");
  Serial.print(desiredPose.x);
  Serial.print(", ");
  Serial.println(desiredPose.y);
  Serial.println("Distance: " + String(dist));
  if (dist <= kPoseArrivalToleranceMm) {
    return 0;
  }

  double ramp = (dist - kPoseArrivalToleranceMm) /
                (kPoseFullSpeedDistanceMm - kPoseArrivalToleranceMm);
  ramp = fmax(0.0, fmin(1.0, ramp));
  double answer = kPoseMinSpeedFactor * pow(kPoseMaxSpeedFactor / kPoseMinSpeedFactor, ramp);
  Serial.print("Speed factor: ");
  Serial.println(answer);
  return answer;
}
