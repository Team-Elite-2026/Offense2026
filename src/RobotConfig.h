#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

enum class RobotMode { Offense, Defense };

constexpr double pincontrolFLA = 22;
constexpr double pincontrolFLB = 23;
constexpr double pinspeedFL    = 2;
constexpr double pincontrolRLA = 18;
constexpr double pincontrolRLB = 31;
constexpr double pinspeedRL    = 4;
constexpr double pincontrolFRA = 20;
constexpr double pincontrolFRB = 21;
constexpr double pinspeedFR    = 3;
constexpr double pincontrolDribblerA = 11;
constexpr double pincontrolDribblerB = 12;
constexpr double pinspeedDribbler    = 6;
constexpr double pincontrolRRA = 9;
constexpr double pincontrolRRB = 10;
constexpr double pinspeedRR    = 5;
constexpr double selectionPin  = 26;

constexpr RobotMode defaultRobotMode = RobotMode::Defense;
constexpr double defenseSpeedFactor  = 0.24;
constexpr double offenseSpeedFactor  = 0.28;
constexpr double lineAvoidanceSpeed  = 0.15;

// Virtual rectangle boundary in the same field coordinate frame as
// Movement::currentPose (millimeters). This acts like an imaginary white line.
constexpr bool virtualBoundaryDebugEnabled = true;
constexpr bool virtualBoundaryDriveEnabled = false;
constexpr double virtualBoundaryMinX = -700.0;
constexpr double virtualBoundaryMaxX = 700.0;
constexpr double virtualBoundaryMinY = -950.0;
constexpr double virtualBoundaryMaxY = 950.0;
constexpr double virtualBoundaryAvoidanceSpeed = 0.15;

#endif
