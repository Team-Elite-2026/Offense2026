#include <Movement.h>
#include <math.h>
#include <trig.h>


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
  double orientationDiff = compassSensor.currentFieldRelativeOffset(goalDirection);
  
  Serial.println("Orientation Diff: " + String(orientationDiff));

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
void Movement::movement(double intended_movement_angle, double speedfactor, double desiredOrientation) {
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
  double correction = findCorrection(desiredOrientation);

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
