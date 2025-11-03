#include <Arduino.h>
#include <LineDetection.h>

LineDetection lineDetection;
// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  double angle = lineDetection.getLineAngle(0, 1);
  Serial.println(angle);

  // put your main code here, to run repeatedly:
}
