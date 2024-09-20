#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT13, ratio18_1, true);
motor leftMotorB = motor(PORT9, ratio18_1, true); 
motor leftMotorC = motor(PORT6, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT1, ratio18_1, false);
motor rightMotorB = motor(PORT2, ratio18_1, false);
motor rightMotorC = motor(PORT12, ratio18_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial dtInertial = inertial(PORT10);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart,dtInertial, 256.74, 295, 40, mm, 1.667);
motor Belt = motor(PORT4, ratio18_1, false);
motor Intake = motor(PORT5, ratio18_1, false);
digital_out Piston = digital_out(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1UpDownButtonsControlMotorsStopped = true;
bool Controller1XBButtonsControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

void togglePiston(){
  if(Piston.value()){
    Piston.set(false);
  }
  else{
    Piston.set(true);
  }
  wait(100,msec);
}
// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  Intake.setVelocity(100,  pct);
  Belt.setVelocity(100, pct);
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();
      Controller1.ButtonR2.pressed(togglePiston);
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonUp/ButtonDown status to control Intake
      if(Controller1.ButtonL2.pressing()){
        Intake.spin(forward);
        Belt.spin(forward);
        Controller1UpDownButtonsControlMotorsStopped = false;
      }
       else if (!Controller1UpDownButtonsControlMotorsStopped) {
        Intake.stop();
        Belt.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1UpDownButtonsControlMotorsStopped = true;
      }
      if(Controller1.ButtonL1.pressing()){
        Intake.spin(reverse);
        Belt.spin(reverse);
        Controller1XBButtonsControlMotorsStopped = false;
      }
      else if (!Controller1XBButtonsControlMotorsStopped) {
        Belt.stop();
        Intake.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1XBButtonsControlMotorsStopped = true;
      }
      /*
      if (Controller1.ButtonUp.pressing()) {
        Intake.spin(forward);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonDown.pressing()) {
        Intake.spin(reverse);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (!Controller1UpDownButtonsControlMotorsStopped) {
        Intake.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1UpDownButtonsControlMotorsStopped = true;
      }
      // check the ButtonX/ButtonB status to control Belt
      if (Controller1.ButtonX.pressing()) {
        Belt.spin(forward);
        Controller1XBButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonB.pressing()) {
        Belt.spin(reverse);
        Controller1XBButtonsControlMotorsStopped = false;
      } else if (!Controller1XBButtonsControlMotorsStopped) {
        Belt.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1XBButtonsControlMotorsStopped = true;
      }
      */
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}