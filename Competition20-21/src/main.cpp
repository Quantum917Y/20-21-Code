/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Team 917Y                                                 */
/*    Created:      Fri Jun 26 2020                                           */
/*    Description:  917Y Change Up Competition Code                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         2               
// BL                   motor         3               
// FR                   motor         4               
// BR                   motor         5               
// IntakeL              motor         6               
// IntakeR              motor         7               
// ElevatorL            motor         8               
// ElevatorR            motor         9               
// Inertial10           inertial      10              
// lEncoder             encoder       A, B            
// rEncoder             encoder       C, D            
// bEncoder             encoder       E, F            
// BallDetector         sonar         G, H            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

motor_group lDrive(FL,BL);
motor_group rDrive(FR, BR);
motor_group fDrive(FL,BL,FR,BR);
motor_group Intakes(IntakeL, IntakeR);
motor_group Elevator(ElevatorL, ElevatorR);

timer drivetimer;

// our drive function (autonomous)
void cosdrive(double inches, double speed){ //uses the changing slope of a cosine wave to accelerate/decelerate the robot for precise movement.
// better explanation and visualization here: https://www.desmos.com/calculator/begor0sggm
	double velocity;
  double seconds=fabs((34.6*inches)/(3.1416*speed*3)); //calculates the time the robot will take to complete a cycle based on the wheel
  //circumference, gear ratio, target distance, and motor speed.
  drivetimer.reset();
  while(drivetimer.time(sec)<seconds){
    velocity=(1-cos((6.283*drivetimer.time(sec))/seconds))*speed/2 * (fabs(inches)/inches); //uses equation for the cosine wave to calculate velocity.
    lDrive.spin(forward,velocity,percent);
		rDrive.spin(forward,velocity,percent);
	}                                                                                                                                                              
	lDrive.stop();
	rDrive.stop();
}

// our turn function (autonomous)
int endAngle=0; //driving forward will drift the back encoder unintentionally,
//so we save the value of where we turned last and use it when turning again to ignore drift.
void Pturn(float angle){ //function for turning. Spins with a speed cap of 36 percent, uses proportional correction
  bEncoder.setPosition(endAngle, degrees);
  float error = angle-(bEncoder.position(degrees)/-5.08);
  while(fabs(error)>2||fabs(bEncoder.velocity(rpm))>1){ //exits loop if error <2 and rotational speed <1
    error = angle-(bEncoder.position(degrees)/-5.08);//calculates error value
    if(fabs(error)>50){ //if error is greater than 50, use proportional correction. if not, turn at 36 percent speed
      lDrive.spin(forward,36*(fabs(error)/error),percent);
      rDrive.spin(reverse,36*(fabs(error)/error),percent);
    }else{
      lDrive.spin(forward,error*0.7,percent);
      rDrive.spin(reverse,error*0.7,percent);
    }
  }
  lDrive.stop();
  rDrive.stop();
  Brain.Screen.print(bEncoder.position(degrees)/-5.08); //print encoder value for debugging
  Brain.Screen.newLine();
  endAngle=bEncoder.position(degrees);
}

void arcade(){ //basic drive where one joystick controls forward/backward motion while the other controls turning. The values are added.
//we use voltage because vex put a speed cap on rpm and percent, and we want to go faster.
  lDrive.spin(forward, (Controller1.Axis3.position()+Controller1.Axis1.position())*0.12, voltageUnits::volt);
  rDrive.spin(forward, (Controller1.Axis3.position()-Controller1.Axis1.position())*0.12, voltageUnits::volt); //*0.12 gets us to a comfortable driving speed.
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  ElevatorL.setBrake(coast);
  ElevatorR.setBrake(coast);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void deploy(){ // raises our guide on our flywheel, moves preload into correct spot, and drops the intakes into position
  //Intakes.spinFor(-160, degrees, 70, rpm, false);
  Intakes.spin(forward,100,percent);
  Elevator.spin(forward,50,percent);
  wait(0.3,sec);
  Elevator.stop();
}

void skills(){
  Intakes.spin(forward,100,percent);         //intakes ball
  cosdrive(25,60);
  Elevator.spinFor(forward, 500, degrees, 300, rpm,false);
  Intakes.stop();
  Pturn(-136);
  cosdrive(31,73);
  Elevator.spinFor(forward,600,degrees,500,rpm); //first goal

  cosdrive(15,-50);
  Pturn(-274);
  cosdrive(46,76);
  Pturn(-183);
  cosdrive(7,30);
  Elevator.spinFor(forward,1000,degrees,600,rpm); //second goal
  cosdrive(7,-30);

  Pturn(-274);
  Intakes.spin(forward,100,percent);
  cosdrive(49,75);
  Intakes.stop();
  Elevator.spinFor(1000,degrees,300,rpm,false);
  Pturn(-230);
  cosdrive(11,40);
  Elevator.spinFor(forward,1000,degrees,530,rpm); //third goal

  cosdrive(47.2,-75);
  Pturn(-371);
  endAngle=0;
  Intakes.spin(forward,100,percent);
  cosdrive(19,52);
  Elevator.spinFor(1300,degrees,300,rpm,false);
  Pturn(89);
  cosdrive(30,62);
  Intakes.stop();
  Elevator.spinFor(600,degrees,500,rpm); //fourth goal

  cosdrive(19,-58);
  Pturn(0);
  Intakes.spin(forward,100,percent);
  cosdrive(44,73);
  Intakes.stop();
  Elevator.spinFor(forward,500,degrees,300,rpm,false);
  Pturn(47);
  cosdrive(23.8,60);
  Elevator.spinFor(800,degrees,500,rpm); //fifth goal

  cosdrive(45.5,-75);
  Pturn(-31);
  cosdrive(27,62);
  Elevator.spinFor(forward,700,degrees,500,rpm); //sixth goal

  cosdrive(8.5,-35);
  Pturn(-91);
  Intakes.spin(forward,100,percent);
  cosdrive(58,78);
  Intakes.stop();
  Elevator.spinFor(forward,800,degrees,300,rpm,false);
  Pturn(-46);
  cosdrive(12,41);
  Elevator.spinFor(forward,1100,degrees,500,rpm); //seventh goal

  cosdrive(46,-75);
  Pturn(-182);
  Intakes.spin(forward,100,percent);
  cosdrive(24,56);
  Intakes.stop();
  Elevator.spinFor(forward,1400,degrees,300,rpm,false);
  Pturn(-272);
  cosdrive(18,60);
  Elevator.spinFor(forward,800,degrees,320,rpm); //eigth goal
  cosdrive(18,-100);
}

void autonomous(void){
  deploy();
  skills();
  

//below is all for testing
  // Inertial10.calibrate();
  // waitUntil(Inertial10.isCalibrating()==false);
  // Pturn(1080);
  Brain.Screen.print(bEncoder.position(degrees)/-5.08);
  Brain.Screen.newLine();
  Brain.Screen.print(Inertial10.yaw());
  Brain.Screen.newLine();
  Controller1.Screen.print(bEncoder.position(degrees)/-5.08);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  FL.setBrake(coast);
  BL.setBrake(coast);
  FR.setBrake(coast);
  BR.setBrake(coast);
   while (1) {
    arcade(); //calls the arcade drive function. we have other drive functions stored in another file so if we want to use those we can switch this function
    Intakes.spin(forward,(Controller1.ButtonL1.pressing()-Controller1.ButtonL2.pressing())*100, percent); 

    if(Controller1.ButtonR1.pressing()&&Controller1.ButtonR2.pressing()){ //if both R1 and R2 are pressed at the same time, the elevator spins slower at a good speed to score the center goal.
      Elevator.spin(forward,420,rpm);
    }else{
      Elevator.spin(forward,(Controller1.ButtonR1.pressing()-Controller1.ButtonR2.pressing())*100, percent);
    }

    Brain.Screen.clearLine();
    Brain.Screen.print(bEncoder.position(degrees)/-5.08);

    // if(Controller1.ButtonY.pressing()){
    //   autonomous();
    // }

    //wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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