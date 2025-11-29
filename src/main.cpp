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
	// if (logLine > 10)
	{
		Brain.Screen.setCursor(1, 0);
		logLine = 0;
	}
	// else
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

inertial InertialSensor = inertial(10, turnType::left);

Drivetrain RobotDrivetrain = Drivetrain(&LeftMotorGroup, &RightMotorGroup, &InertialSensor, 0.5f, -0.3f, 1.0f, 0.1f, 0.01f, 0.05f, Log);

#pragma region DriveDef

#pragma endregion

void pre_auton()
{
	InertialSensor.calibrate(2);
	while (InertialSensor.isCalibrating())
	{
		wait(100, timeUnits::msec);
	}
	InertialSensor.setHeading(0, rotationUnits::deg);
}

#pragma region Auton

void autonomous(void)
{
	// for (int i = 0; i < 26; i++)
	{
		Log("test");
		wait(500, timeUnits::msec);
	}

	RobotDrivetrain.TurnToDegPID(180);
}

#pragma endRegion

void drivercontrol(void)
{
	// User control code here, inside the loop
	while (1)
	{

		if (Controller.ButtonUp.pressing())
		{
			LeftMotorGroup.setVelocity(100, percentUnits::pct);
			RightMotorGroup.setVelocity(100, percentUnits::pct);
			LeftMotorGroup.setMaxTorque(100, percentUnits::pct);
			RightMotorGroup.setMaxTorque(100, percentUnits::pct);
		}
		else if (Controller.ButtonDown.pressing())
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
	}
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
	// Set up callbacks for autonomous and driver control periods.
	Competition.autonomous(autonomous);
	Competition.drivercontrol(drivercontrol);

	// Run the pre-autonomous function.
	pre_auton();
	autonomous();
	// Prevent main from exiting with an infinite loop.
	while (true)
	{
		wait(100, msec);
	}
}
