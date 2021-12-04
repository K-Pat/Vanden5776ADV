#include "main.h"

using namespace okapi;

extern Motor leftBack;
extern Motor leftFront;
extern Motor rightBack;
extern Motor rightFront;

void updateDrive();

void translatePID(double distance, int ms);

void rotatePID(double turnDegrees, int ms);
