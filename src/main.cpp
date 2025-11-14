#include "vex.h"

using namespace vex;
competition Competition;
controller Controller;


motor_group LeftMotorGroup = motor_group(
  motor(0, vex::gearSetting::ratio6_1, true), 
  motor(1, vex::gearSetting::ratio6_1, false), 
  motor(2, vex::gearSetting::ratio6_1, false));

motor_group RightMotorGroup = motor_group(
  motor(3, vex::gearSetting::ratio6_1, true),
  motor(4, vex::gearSetting::ratio6_1, false), 
  motor(5, vex::gearSetting::ratio6_1, false));

motor LowerAgitator = motor(7, vex::gearSetting::ratio36_1);
motor UpperAgitator = motor(8, vex::gearSetting::ratio36_1);
motor LowerIntake = motor(9, vex::gearSetting::ratio36_1);
motor UpperOuttake = motor(10, vex::gearSetting::ratio36_1);

digital_out Unloader = digital_out(Brain.ThreeWirePort.A);

#pragma region DriveDef
Drive chassis(
ZERO_TRACKER_ODOM,
//Left Motors:
LeftMotorGroup,
//Right Motors:
RightMotorGroup,
//The PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
6,
//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,
//External ratio, must be in decimal, in the format of input teeth/output teeth.
0.6,
//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.
//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,
//LB:      //RB: 
PORT3,     -PORT4,
//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,
//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,
//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,
//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,
//Sideways tracker diameter (reverse to make the direction switch):
-2.75,
//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5
);
#pragma endregion

void pre_auton() {

}


void autonomous(void) {

}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    float Throttle = Controller.Axis1.position();
    float Steer = Controller.Axis2.position();

    LeftMotorGroup.spin(directionType::fwd, Throttle + Steer, velocityUnits::pct);
    RightMotorGroup.spin(directionType::fwd, Throttle - Steer, velocityUnits::pct);

    if (Controller.ButtonL2.pressing())
    {
      LowerIntake.spin(directionType::fwd);
      LowerAgitator.spin(directionType::rev);
      UpperAgitator.spin(directionType::rev);
      UpperOuttake.spin(directionType::rev);
    }
    else if (Controller.ButtonR2.pressing())
    {
      LowerIntake.spin(directionType::fwd);
      LowerAgitator.spin(directionType::fwd);
      UpperAgitator.spin(directionType::fwd);
      UpperOuttake.spin(directionType::rev);
    }
    else if (Controller.ButtonR1.pressing())
    {
      LowerIntake.spin(directionType::fwd);
      LowerAgitator.spin(directionType::fwd);
      UpperAgitator.spin(directionType::fwd);
      UpperOuttake.spin(directionType::fwd);
    }
    else if (Controller.ButtonR2.pressing())
    {
      LowerIntake.spin(directionType::rev);
      LowerAgitator.spin(directionType::fwd);
      UpperAgitator.spin(directionType::fwd);
      UpperOuttake.spin(directionType::fwd);
    }
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
