using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor FL;
extern motor BL;
extern motor FR;
extern motor BR;
extern motor IntakeL;
extern motor IntakeR;
extern motor ElevatorL;
extern motor ElevatorR;
extern inertial Inertial10;
extern encoder lEncoder;
extern encoder rEncoder;
extern encoder bEncoder;
extern sonar BallDetector;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );