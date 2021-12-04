#include "main.h"

using namespace okapi;

Motor twoBarLift(twoBarLiftPort, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

int twoBarButtonCount;

typedef struct PID pid;

pid TB;

double twoBarPID(double setpoint)
{
  TB.kP = 0.6;
  TB.kI = 0;
  TB.kD = 0.09;

  TB.target = setpoint;
  TB.error = TB.target - twoBarLift.getPosition();
  TB.integral += TB.error;
  TB.derivative = TB.error - TB.prevError;
  TB.power = TB.kP * TB.error + TB.kI * TB.integral + TB.kD * TB.derivative;

  return TB.power;
}

void autonTwoBarPID(double setpoint)
{
  while (true)
  {
    TB.kP = 0.5;
    TB.kI = 0.001;
    TB.kD = 0.05;

    TB.target = setpoint;
    TB.error = TB.target - twoBarLift.getPosition();
    TB.integral += TB.error;
    TB.derivative = TB.error - TB.prevError;
    TB.power = TB.kP * TB.error + TB.kI * TB.integral + TB.kD * TB.derivative;

    twoBarLift.moveVelocity(TB.power);

    if (abs(TB.error) < 7)
    {
      twoBarLift.setBrakeMode(AbstractMotor::brakeMode::hold);
      break;
    }

    pros::delay(10);
  }
}

void updateTwoBarLiftMacro()
{
  if (controller.getDigital(ControllerDigital::X) == 1)
  {
    twoBarButtonCount = 2;
  }
  else if(controller.getDigital(ControllerDigital::B) == 1)
  {
    twoBarButtonCount = 1;
  }

  switch (twoBarButtonCount)
  {
  case 1:
    twoBarLift.moveVelocity(twoBarPID(-470));
    break;
  case 2:
    twoBarLift.moveVelocity(twoBarPID(-300));
    break;
  }

}
