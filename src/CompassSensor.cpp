#include <CompassSensor.h>
#include <Wire.h>
#include <trig.h>

CompassSensor::CompassSensor() {
    
}

void CompassSensor::begin() {
  Wire.begin();
    bno = Adafruit_BNO055(55, 0x28, &Wire2);
    if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } else {
    Serial.print("Successful");
  }
}

void CompassSensor::callibrate() {
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    while (mag<3) {
        Serial.print("Mag: ");
        Serial.print(mag);
        Serial.println("/3");
        delay(500);
        bno.getCalibration(&system, &gyro, &accel, &mag);
    }
}

// returns a value between 0 and 360
int CompassSensor::getOrientation() {
    bno.getEvent(&event);
    return event.orientation.x;
}

// range between -180 and 180
// this returns the offset between the current orientation and the zeroed angle
int CompassSensor::currentOffset() {
  int offset = getOrientation() - this->zeroedAngle;
  // Serial.println("Current Orientation: " + String(getOrientation()));
  // Serial.println("Zeroed Angle: " + String(this->zeroedAngle));
  // Serial.println("Calculated offset: - Orientation Diff: " + String(offset));

  return Trig::wrapAngle(offset);

}

// goalAngle is field-relative from zeroed heading (0 = zeroed angle).
// Returns signed heading error in [-180, 180] as (current - target).
int CompassSensor::currentFieldRelativeOffset(double goalAngle) {
  int currentOrientation = getOrientation();
  double targetOrientation = this->zeroedAngle + goalAngle;
  double offset = currentOrientation - targetOrientation;

  return static_cast<int>(Trig::wrapAngle(offset));
}

int CompassSensor::currentOffset(double goalAngle) {
  return currentFieldRelativeOffset(goalAngle);
}

// Converts a robot-relative target to a field-relative heading target.
// Assumes matching sign convention between relative angles and compass heading.
double CompassSensor::robotRelativeToField(double robotRelativeAngle) {
  return Trig::wrapAngle(currentOffset() + robotRelativeAngle);
}
