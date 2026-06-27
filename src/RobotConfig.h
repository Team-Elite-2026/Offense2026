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

constexpr RobotMode defaultRobotMode = RobotMode::Offense;
constexpr double defenseSpeedFactor  = 0.26;
constexpr double offenseSpeedFactor  = 0.22;
constexpr double lineAvoidanceSpeed  = 0.15;

#endif
