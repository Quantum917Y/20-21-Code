#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FL = motor(PORT2, ratio36_1, true);
motor BL = motor(PORT3, ratio36_1, true);
motor FR = motor(PORT4, ratio36_1, false);
motor BR = motor(PORT5, ratio36_1, false);
motor IntakeL = motor(PORT6, ratio18_1, true);
motor IntakeR = motor(PORT7, ratio18_1, false);
motor ElevatorL = motor(PORT8, ratio6_1, true);
motor ElevatorR = motor(PORT9, ratio6_1, false);
inertial Inertial10 = inertial(PORT10);
encoder lEncoder = encoder(Brain.ThreeWirePort.A);
encoder rEncoder = encoder(Brain.ThreeWirePort.C);
encoder bEncoder = encoder(Brain.ThreeWirePort.E);
sonar BallDetector = sonar(Brain.ThreeWirePort.G);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}