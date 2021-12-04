#include "main.h"

using namespace okapi;

extern Motor twoBarLift;

void updateTwoBarLiftMacro();

double twoBarPID(double setpoint);

void autonTwoBarPID(double setpoint);
