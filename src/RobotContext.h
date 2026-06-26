#ifndef ROBOT_CONTEXT_H
#define ROBOT_CONTEXT_H

#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>
#include <Movement.h>
#include <orbit.h>
#include <Cam.h>
#include <LinePCBComm.h>
#include <ModeControl.h>

extern RobotMode kRobotMode;

extern CompassSensor compassSensor;
extern Switch switches;
extern Calibration calibration;
extern Motor* FL;
extern Motor* FR;
extern Motor* BL;
extern Motor* BR;
extern Movement* movement;
extern Orbit orbit;
extern Cam camera;
extern LinePCBComm linePCBComm;
extern ModeControl* modeControl;

extern double lineAngle;
extern double currentOffset;
extern double orbitAngle;
extern double maxChordLength;
extern double goalAngle;
extern double avoidanceAngle;
extern bool aimingGoal;

void initializeRobotContext();

#endif
