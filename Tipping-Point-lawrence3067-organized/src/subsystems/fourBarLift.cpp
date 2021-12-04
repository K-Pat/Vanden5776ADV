#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarLiftPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor fourBarLift2(fourBarLiftPort2, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

int liftButtonCount;

typedef struct PID pid;

pid FB;

double fourBarPID(double setpoint)
{
  FB.kP = 0.5;
  FB.kI = 0;
  FB.kD = 0.05;

  FB.target = setpoint;
  FB.error = FB.target - fourBarLift.getPosition();
  FB.integral += FB.error;
  FB.derivative = FB.error - FB.prevError;
  FB.power = FB.kP * FB.error + FB.kI * FB.integral + FB.kD * FB.derivative;

  return FB.power;
}

void autonFourBarPID(double setpoint)
{
  while (true)
  {
    FB.kP = 0.85;
    FB.kI = 0;
    FB.kD = 0.05;

    FB.target = setpoint;
    FB.error = FB.target - fourBarLift.getPosition();
    FB.integral += FB.error;
    FB.derivative = FB.error - FB.prevError;
    FB.power = FB.kP * FB.error + FB.kI * FB.integral + FB.kD * FB.derivative;

    fourBarLift.moveVelocity(FB.power);
    fourBarLift2.moveVelocity(FB.power);

    if (abs(FB.error) < 7)
    {
      fourBarLift.setBrakeMode(AbstractMotor::brakeMode::hold);
      fourBarLift2.setBrakeMode(AbstractMotor::brakeMode::hold);

      break;
    }

    pros::delay(10);
  }
}

void updateFourBarLiftMacro()
{
  if (controller.getDigital(ControllerDigital::L1) == 1)
  {
    liftButtonCount = 1;
  }

  else if(controller.getDigital(ControllerDigital::L2) == 1)
  {
    liftButtonCount = 2;
  }

  if (controller.getDigital(ControllerDigital::A) == 1)
  {
    liftButtonCount = 3;
  }

  if (controller.getDigital(ControllerDigital::B) == 1)
  {
    liftButtonCount = 4;
  }

  switch (liftButtonCount)
  {
  case 1:
    fourBarLift.moveVelocity(fourBarPID(-800));
    fourBarLift2.moveVelocity(fourBarPID(-800));
    break;
  case 2:
    fourBarLift.moveVelocity(fourBarPID(50));
    fourBarLift2.moveVelocity(fourBarPID(50));
    break;
  case 3:
    fourBarLift.moveVelocity(fourBarPID(-450));
    fourBarLift2.moveVelocity(fourBarPID(-450));
    rightFront.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftFront.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    break;
  case 4:
    fourBarLift.moveVelocity(fourBarPID(-550));
    fourBarLift2.moveVelocity(fourBarPID(-550));
    break;
  }
}
