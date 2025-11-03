#include <CompassSensor.h>
#include <Wire.h>

CompassSensor::CompassSensor() {
    Wire.begin();
    bno = Adafruit_BNO055(55, 0x28, &Wire);
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

int CompassSensor::getOrientation() {
    bno.getEvent(&event);
    return event.orientation.x;
}

// range between -180 and 180
int CompassSensor::currentOffset() {
  int offset = getOrientation() - this->zeroedAngle;
  // Serial.println("Current Orientation: " + String(getOrientation()));
  // Serial.println("Zeroed Angle: " + String(this->zeroedAngle));
  // Serial.println("Calculated offset: - Orientation Diff: " + String(offset));

  if (offset < -180) {
    return offset + 360; 
  } else if (offset > 180) {
    return offset - 360; 
  } else {
    return offset; 
  }

}

int CompassSensor::currentOffset(double goalAngle) {
  int currentOrientation = getOrientation();
  int offset = currentOrientation - (this->zeroedAngle + goalAngle);

  if (offset < -180) {
    return offset + 360; 
  } else if (offset > 180) {
    return offset - 360; 
  } else {
    return offset; 
  }
}

