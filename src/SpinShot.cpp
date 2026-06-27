#include <SpinShot.h>

SpinShot::SpinShot() {}

int SpinShot::goToPointAngle(Point ourPosition, Point targetPosition) {
    if(Trig::getDist(ourPosition, targetPosition) < 5) {
        return -5;
    }
    double angle = 
}