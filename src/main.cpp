#include "vex.h"
#include <sstream>
#include <string>
#include <math.h>
#include "Drivetrain.h"
using namespace vex;

competition Competition;
controller Controller;
brain Brain;

int totalLine = 0;
int logLine = 0;
void Log(std::string msg)
{
  std::stringstream stream;
  stream << "[" << totalLine << "][" << Brain.timer(timeUnits::sec) << "] " << msg;
  Brain.Screen.clearLine();
  Brain.Screen.print(stream.str().c_str());
  Brain.Screen.newLine();
  Brain.Screen.clearLine();
  logLine++;
  totalLine++;
  //if (logLine > 10)
  {
    Brain.Screen.setCursor(1, 0);
    logLine = 0;
  }
  //else
  {
  //  Brain.Screen.print("-----------------------------------------------");
  }
}

motor FrontLeft = motor(4, vex::gearSetting::ratio6_1, false);
motor MiddleLeft = motor(6, vex::gearSetting::ratio6_1, false);
motor RearLeft = motor(8, vex::gearSetting::ratio6_1, false);

motor FrontRight = motor(5, vex::gearSetting::ratio6_1, false);
motor MiddleRight = motor(7, vex::gearSetting::ratio6_1, false);
motor RearRight = motor(9, vex::gearSetting::ratio6_1, false);

motor_group LeftMotorGroup = motor_group(
	FrontLeft,
	MiddleLeft, 
	RearLeft);

motor_group RightMotorGroup = motor_group(
	FrontRight,
	MiddleRight, 
	RearRight);

motor LowerAgitator = motor(1, vex::gearSetting::ratio36_1);
motor UpperAgitator = motor(0, vex::gearSetting::ratio36_1);
motor LowerIntake = motor(3, vex::gearSetting::ratio36_1);
motor UpperOuttake = motor(2, vex::gearSetting::ratio36_1);

digital_out Unloader = digital_out(Brain.ThreeWirePort.A);

inertial InertialSensor = inertial(10, turnType::left);

Drivetrain RobotDrivetrain = Drivetrain(&LeftMotorGroup, &RightMotorGroup, &InertialSensor, 0.5f, -0.3f, 1.0f, Log);

#pragma region DriveDef



#pragma endregion

void pre_auton() {
  InertialSensor.calibrate(2);
  while (InertialSensor.isCalibrating())
  {
    wait(100, timeUnits::msec);
  }
}

#pragma region Auton

void Load()
{
  LowerIntake.spin(directionType::fwd);
  LowerAgitator.spin(directionType::rev);
  UpperAgitator.spin(directionType::rev);
  UpperOuttake.spin(directionType::rev);
}

void Unload()
{
  LowerIntake.spin(directionType::rev);
  LowerAgitator.spin(directionType::fwd);
  UpperAgitator.spin(directionType::fwd);
  UpperOuttake.spin(directionType::fwd);
}

void LowGoal()
{
  LowerIntake.spin(directionType::fwd);
  LowerAgitator.spin(directionType::fwd);
  UpperAgitator.spin(directionType::fwd);
  UpperOuttake.spin(directionType::rev);
}

void HighGoal()
{
  LowerIntake.spin(directionType::fwd);
  LowerAgitator.spin(directionType::fwd);
  UpperAgitator.spin(directionType::fwd);
  UpperOuttake.spin(directionType::fwd);
}

void move(float dist)
{
  LeftMotorGroup.spinFor(dist, rotationUnits::rev, false);
  RightMotorGroup.spinFor(dist, rotationUnits::rev, true);
}

void turn(float dist)
{

}

void autonomous(void) {
  //for (int i = 0; i < 26; i++)
  {
    Log("test");
    wait(500, timeUnits::msec);
  }
  

  RobotDrivetrain.TurnCommandDegPID(180);
}


#pragma endRegion

bool UpperJam = false;
bool LowerJam = false;
bool UpperSpin = false;
bool LowerSpin = false;
void drivercontrol(void) {
  // User control code here, inside the loop 
  while (1) {

    if (Controller.ButtonUp.pressing())
    {
      LeftMotorGroup.setVelocity(100, percentUnits::pct);
      RightMotorGroup.setVelocity(100, percentUnits::pct);
      LeftMotorGroup.setMaxTorque(100, percentUnits::pct);
      RightMotorGroup.setMaxTorque(100, percentUnits::pct);
    } else if (Controller.ButtonDown.pressing())
    {
      LeftMotorGroup.setVelocity(40, percentUnits::pct);
      RightMotorGroup.setVelocity(40, percentUnits::pct);
      LeftMotorGroup.setMaxTorque(40, percentUnits::pct);
      RightMotorGroup.setMaxTorque(40, percentUnits::pct);
    }

    float Throttle = Controller.Axis3.position();
    float Steer = Controller.Axis1.position();

    LeftMotorGroup.spin(directionType::fwd, Steer + Throttle, velocityUnits::pct);
    RightMotorGroup.spin(directionType::fwd, Steer - Throttle, velocityUnits::pct);

    UpperSpin = true;
    LowerSpin = true;
    
    if (Controller.ButtonL2.pressing())
    {
      LowerIntake.spin(directionType::fwd);
      LowerAgitator.spin(directionType::rev); 
      /*if (!UpperAgitator.isSpinning())*/ UpperAgitator.spin(directionType::rev);
      UpperOuttake.spin(directionType::rev);
    }
    else if (Controller.ButtonR2.pressing())
    {
      LowerIntake.spin(directionType::fwd);
      LowerAgitator.spin(directionType::fwd);
      /*if (!UpperAgitator.isSpinning())*/ UpperAgitator.spin(directionType::fwd);
      UpperOuttake.spin(directionType::rev);
    }
    else if (Controller.ButtonR1.pressing())
    {
      LowerIntake.spin(directionType::fwd);
      LowerAgitator.spin(directionType::fwd);
      /*if (!UpperAgitator.isSpinning())*/ UpperAgitator.spin(directionType::fwd);
      UpperOuttake.spin(directionType::fwd);
    }
    else if (Controller.ButtonL1.pressing())
    {
      LowerIntake.spin(directionType::rev);
      LowerAgitator.spin(directionType::fwd);
      /*if (!UpperAgitator.isSpinning())*/ UpperAgitator.spin(directionType::fwd);
      UpperOuttake.spin(directionType::fwd);
    }
    else
    {
      UpperSpin = false;
      LowerSpin = false;
      LowerIntake.stop();
      LowerAgitator.stop();
      UpperAgitator.stop();
      UpperOuttake.stop();
    }
    /*
    if (UpperSpin && UpperAgitator.velocity(percentUnits::pct) < 10)
    {
      UpperJam = true;
      if (UpperAgitator.direction() == directionType::fwd)
      {
        UpperAgitator.spinFor(directionType::fwd, 2, rotationUnits::rev, false);
      }
      else
      {
        UpperAgitator.spinFor(directionType::rev, 2, rotationUnits::rev, false);
      }
      
    }
    */

    if (Controller.ButtonA.pressing())
    {
      Unloader.set(true);
    }
    else if (Controller.ButtonB.pressing())
    {
      Unloader.set(false);
    }
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  autonomous();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
