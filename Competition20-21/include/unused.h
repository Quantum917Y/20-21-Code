#include "vex.h"

void drive(int direction, int speed, int distance, double time=0){ //direction determines how sharp of a turn, distance is measured in degrees based on the motor, if time is filled in the function will stop based on the time limit.
  lDrive.setPosition(0, degrees);
  rDrive.setPosition(0, degrees);
  if(time==0){
    while((fabs(lDrive.position(degrees))+fabs(rDrive.position(degrees)))/2 < distance){
      lDrive.spin(forward, speed + direction*speed/100, percent);
      rDrive.spin(forward, speed - direction*speed/100, percent);
    }
  }else{
    lDrive.spin(forward,(speed+direction*speed/100)*6,rpm);
    rDrive.spinFor(time,sec,(speed-direction*speed/100)*6,rpm);
  }
  lDrive.stop();
  rDrive.stop();
}

void cosdrive(double seconds, double speed){
  double velocity;
  drivetimer.reset();
  while(drivetimer.time(sec)<seconds){
    velocity=(1-cos((6.283*drivetimer.time(sec))/seconds))*speed/2;
    lDrive.spin(forward,velocity,percent);
		rDrive.spin(forward,velocity,percent);
	}                                                                                                                                                              
	lDrive.stop();
	rDrive.stop();
}

void costurn(int angle, int speed){ //uses the slope of a cosine wave to accelerate/decelerate the robot for more precision
  bEncoder.resetRotation();
  double velocity;
  double seconds=fabs((angle*7.2)/(speed*18)); //  7.2/18 is our ratio to convert distance/speed to time
  drivetimer.reset();
  while(drivetimer.time(sec)<seconds){
    velocity=(1-cos((6.283*drivetimer.time(sec))/seconds))*speed/2 /*equation for cosine wave*/ * (abs(angle)/angle);//determines if angle is negative to make speed negative
    lDrive.spin(forward,velocity,percent);
		rDrive.spin(forward,0-velocity,percent);
	}
  // if(fabs(angle-(bEncoder.position(degrees)/-5))>2){
  //   angle=angle-(bEncoder.position(degrees)/-5);
  //   Brain.Screen.print(bEncoder.position(degrees)/-5);
  //   speed=10*(abs(angle)/angle);
  //   seconds=fabs((angle*7.3)/(speed*18))*10;
  //   drivetimer.reset();
  //   while(drivetimer.time(sec)<seconds){
  //     velocity=(1-cos((6.283*drivetimer.time(sec))/seconds))*speed/2;
  //     lDrive.spin(forward,velocity,percent);
	// 	  rDrive.spin(forward,0-velocity,percent);
  //     Brain.Screen.printAt(1,20,"f% %%",bEncoder.position(degrees)/-5);
  //   }
  // }
	lDrive.stop();
	rDrive.stop();
}

void gdrive(double error=1, double p=1){ // a drive based on the facing of the joystick and the facing of the robot. The robot will point in whichever direction the jostick points in, then will move forwards.
   error=(atan2(Controller1.Axis1.position(),Controller1.Axis2.position())*57.3)-Inertial10.yaw();
   if(Controller1.Axis1.position()==0&&Controller1.Axis2.position()==0){
     lDrive.stop();
     rDrive.stop();
   }else if(fabs(error)<=180){
     p=error*(10/18);
     lDrive.spin(forward,p*0.5,percent);
     rDrive.spin(reverse,p*0.5,percent);
   }else{
     p=(error-(360*(fabs(error)/error)));
     lDrive.spin(forward,p*0.5,percent);
     rDrive.spin(reverse,p*0.5,percent);
 }
}

void PDturn(int angle){ //a PD-driven turn function. It uses the derivative of the turning speed to reduce speed and overshooting
  float error = angle-(bEncoder.position(degrees)/-5); // (encoder/-5) is the direction of the robot (degrees)
  float previouserror = error;
  float d = error-previouserror;
  float kP=0.7;
  float kD=1;
  while(fabs(error)>2||fabs(bEncoder.velocity(rpm))>1){
    previouserror=error;
    wait(20,msec);
    error=angle-(bEncoder.position(degrees)/-5);
    d=error-previouserror;
    lDrive.spin(forward,(kP*error)+(kD*d),percent);
    rDrive.spin(reverse,(kP*error)+(kD*d),percent);
  }
  lDrive.stop();
  rDrive.stop();
}

void auton1(){ //3 goal 15 second auton
  drive(0, 30, 250);
  Elevator.spinFor(300, degrees, 500, rpm);
  wait(0.1, sec);
  drive(-50, 20, 310);
  Elevator.spinFor(2300, degrees, 600, rpm);
  Intakes.stop();
  drive(-8, -41, 3950);
  wait(0.5, sec);
  drive(-200, 17, 260);
  wait(0.35, sec);
  drive(0, 30, 1400,1.6);
  Intakes.spin(forward, 200, rpm);
  Elevator.spinFor(2050, degrees, 600, rpm);
  Intakes.spin(reverse, 200, rpm);
  drive(0, -40, 300);
  Intakes.stop();
  wait(0.2, sec);
  drive(-60, -27, 550);
  drive(0, -40, 910);
  drive(100, -12, 920);
  Intakes.spin(forward, 600, rpm);
  drive(0, 65, 3510);
  wait(0.1, sec);
  Elevator.spinFor(2000, degrees, 600, rpm);
  // Intakes.spin(reverse, 600, rpm);
  // drive(0, -65, 200);
  Intakes.stop();
}

void oldskills(){ //26 point autonomous, NW W SW S SE goals
  Intakes.spin(forward,100,percent);         //intakes ball
  cosdrive(24,50);
  Elevator.spinFor(forward, 500, degrees, 500, rpm,false);
  Intakes.stop();
  Pturn(-135);
  cosdrive(31.8,60);
  Intakes.spin(forward,100,percent);             // first goal +4 points (Southeast)
  fDrive.spin(forward,10,percent);
  Elevator.spinFor(forward,2550,degrees,500,rpm);
  fDrive.stop();
  Intakes.spin(reverse,100,percent);

  cosdrive(16,-50);
  Elevator.spinFor(forward,-1750, degrees, 500, rpm); //dumps blue balls
  Intakes.stop();
  Pturn(22.6);
  Intakes.spin(forward,100,percent);
  cosdrive(52.5,75);                                //intakes ball
  Elevator.spinFor(forward, 1375, degrees,500,rpm,false);
  Pturn(-95.5);
  cosdrive(32.5, 50);                             //intake ball
  fDrive.spin(forward,10,percent);
  Elevator.spinFor(forward,1700,degrees,500,rpm);      //second goal +2 points (South)
  fDrive.stop();
  Intakes.stop();

  cosdrive(19,-50);
  Intakes.spin(reverse,100,percent);
  Elevator.spinFor(reverse,1000,degrees,600,rpm);     //dumps blue ball
  Intakes.stop();
  Pturn(-3.5);
  Intakes.spin(forward,100,percent);                 //intakes ball
  cosdrive(44,65);
  Elevator.spinFor(forward,600,degrees,500,rpm);
  Intakes.stop();
  cosdrive(10,-30);
  Pturn(-48);
  cosdrive(34,60);                               //third goal +9 points (Southwest)
  Intakes.spin(forward,100,percent);
  fDrive.spin(forward,10,percent);
  Elevator.spinFor(forward,2500,degrees,500,rpm);
  fDrive.stop();
  Intakes.spin(reverse,100,percent);

  cosdrive(33,-50);
  Pturn(-180);
  Elevator.spinFor(reverse,1500,degrees,500,rpm); //dumps blue balls
  Intakes.stop();
  Pturn(-90);
  Intakes.spin(forward,100,percent);               //intakes ball
  cosdrive(22.5,50);
  Elevator.spinFor(forward,1000,degrees,500,rpm,false);
  Pturn(-270);                                  //backs against wall to straighten
  Intakes.stop();
  cosdrive(8,-30);
  endAngle=0;                                  //resets position
  cosdrive(6,30);
  Pturn(8.4);
  Intakes.spin(forward,90,percent);              //intakes ball
  cosdrive(55,85);
  Elevator.spinFor(forward,500, degrees,500,rpm);
  Intakes.stop();
  Pturn(-94.5);
  cosdrive(31,55);                                //fourth goal +2 points (West)
  Intakes.spin(forward,100,percent);
  fDrive.spin(forward,10,percent);
  Elevator.spinFor(forward,1600,degrees,500,rpm);    
  fDrive.stop(); 

  Elevator.spinFor(reverse,400,degrees,500,rpm,false);
  cosdrive(8,-30);
  Intakes.spin(reverse,50,percent);
  Elevator.spinFor(reverse,1000,degrees,500,rpm);     //dumps blue ball
  Intakes.stop();
  Pturn(-4);
  Intakes.spin(forward,100,percent);             //intakes ball
  cosdrive(47,65);
  Elevator.spinFor(250,degrees,500,rpm,false);
  Pturn(-49);
  Intakes.stop();
  cosdrive(17,40);                             //fifth goal +9 points (Northwest)
  Intakes.spin(forward,100,percent);
  fDrive.spin(forward,10,percent);
  Elevator.spinFor(forward,1900,degrees,500,rpm);
  fDrive.stop();
  Intakes.stop();
  Elevator.spinFor(forward,600,degrees,500,rpm);
  cosdrive(16,-50);
}                                                  //26 points

void skills2(){
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
  cosdrive(8.3,30);
  Elevator.spinFor(forward,1000,degrees,600,rpm); //second goal
  cosdrive(7,-30);

  Pturn(-270);
  Intakes.spin(forward,100,percent);
  cosdrive(49,75);
  Intakes.stop();
  Elevator.spinFor(1000,degrees,300,rpm,false);
  Pturn(-229);
  cosdrive(10,40);
  Elevator.spinFor(forward,1000,degrees,550,rpm); //third goal

  cosdrive(46.5,-75);
  Pturn(-361.2);
  endAngle=0;
  Intakes.spin(forward,100,percent);
  cosdrive(22.4,55);
  Elevator.spinFor(1300,degrees,300,rpm,false);
  Pturn(86);
  cosdrive(29.5,62);
  Intakes.stop();
  Elevator.spinFor(600,degrees,500,rpm); //fourth goal

  cosdrive(17,-58);
  Pturn(-3.3);
  Intakes.spin(forward,100,percent);
  cosdrive(44.5,73);
  Intakes.stop();
  Elevator.spinFor(forward,500,degrees,300,rpm,false);
  Pturn(51.5);
  cosdrive(24.8,60);
  Elevator.spinFor(800,degrees,500,rpm); //fifth goal

  cosdrive(45.5,-75);
  Pturn(-31);
  cosdrive(28.2,62);
  Elevator.spinFor(forward,700,degrees,500,rpm); //sixth goal

  cosdrive(8.5,-35);
  Pturn(-92.5);
  Intakes.spin(forward,100,percent);
  cosdrive(58,78);
  Intakes.stop();
  Elevator.spinFor(forward,800,degrees,300,rpm,false);
  Pturn(-47);
  cosdrive(12.5,41);
  Elevator.spinFor(forward,1100,degrees,500,rpm); //seventh goal

  cosdrive(45,-75);
  Pturn(-182);
  Intakes.spin(forward,100,percent);
  cosdrive(28,60);
  Intakes.stop();
  Elevator.spinFor(forward,1400,degrees,300,rpm,false);
  Pturn(-272);
  cosdrive(18,60);
  Elevator.spinFor(forward,800,degrees,340,rpm); //eigth goal
  cosdrive(18,-100);
}

void skills_noendagnle(){
  Intakes.spin(forward,100,percent);         //intakes ball
  cosdrive(25,60);
  Elevator.spinFor(forward, 500, degrees, 300, rpm,false);
  Intakes.stop();
  Pturn(-137);
  cosdrive(31,73);
  Elevator.spinFor(forward,600,degrees,500,rpm); //first goal

  cosdrive(15,-50);
  Pturn(-274);
  cosdrive(46,76);
  Pturn(-187);
  cosdrive(7,27);
  Elevator.spinFor(forward,1000,degrees,600,rpm); //second goal
  cosdrive(6,-25);

  Pturn(-277);
  Intakes.spin(forward,100,percent);
  cosdrive(49,75);
  Intakes.stop();
  Elevator.spinFor(1000,degrees,300,rpm,false);
  Pturn(-236);
  cosdrive(12,40);
  Elevator.spinFor(forward,1000,degrees,550,rpm); //third goal

  cosdrive(49,-75);
  Pturn(-365);
  bEncoder.resetRotation();
  Intakes.spin(forward,100,percent);
  cosdrive(21,55);
  Elevator.spinFor(1300,degrees,300,rpm,false);
  Pturn(85);
  cosdrive(30,62);
  Intakes.stop();
  Elevator.spinFor(600,degrees,500,rpm); //fourth goal

  cosdrive(18,-58);
  Pturn(-4);
  Intakes.spin(forward,100,percent);
  cosdrive(44.5,73);
  Intakes.stop();
  Elevator.spinFor(forward,500,degrees,300,rpm,false);
  Pturn(42);
  cosdrive(24,60);
  Elevator.spinFor(800,degrees,500,rpm); //fifth goal

  cosdrive(45.5,-75);
  Pturn(-38);
  cosdrive(24,62);
  Elevator.spinFor(forward,700,degrees,500,rpm); //sixth goal

  cosdrive(8.5,-35);
  Pturn(-98);
  Intakes.spin(forward,100,percent);
  cosdrive(58,78);
  Intakes.stop();
  Elevator.spinFor(forward,800,degrees,300,rpm,false);
  Pturn(-56);
  cosdrive(10.5,41);
  Elevator.spinFor(forward,1100,degrees,500,rpm); //seventh goal

  cosdrive(45,-75);
  Pturn(-184);
  Intakes.spin(forward,100,percent);
  cosdrive(28,60);
  Intakes.stop();
  Elevator.spinFor(forward,1400,degrees,300,rpm,false);
  Pturn(-275);
  cosdrive(18,60);
  Elevator.spinFor(forward,800,degrees,340,rpm); //eigth goal
  cosdrive(18,-100);
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
  cosdrive(47,74);
  Pturn(-183);
  cosdrive(6.2,27);
  Elevator.spinFor(forward,1000,degrees,600,rpm); //second goal


  cosdrive(7,-30);
  Pturn(-275);
  Intakes.spin(forward,100,percent);
  cosdrive(47,75);
  Intakes.stop();
  Elevator.spinFor(1000,degrees,300,rpm,false);
  Pturn(-233);
  cosdrive(10,40);
  Elevator.spinFor(forward,1000,degrees,530,rpm); //third goal

  cosdrive(49.5,-75);
  Pturn(-365);
  endAngle=0;
  Intakes.spin(forward,100,percent);
  cosdrive(19,52);
  Elevator.spinFor(1300,degrees,300,rpm,false);
  Pturn(86);
  cosdrive(28,62);
  Intakes.stop();
  Elevator.spinFor(600,degrees,500,rpm); //fourth goal

  cosdrive(19,-50);
  Pturn(-3.5);
  Intakes.spin(forward,100,percent);
  cosdrive(44.5,73);
  Intakes.stop();
  Elevator.spinFor(forward,500,degrees,300,rpm,false);
  Pturn(50);
  cosdrive(24.7,59);
  Elevator.spinFor(800,degrees,500,rpm); //fifth goal

  cosdrive(46.4,-75);
  Pturn(-31);
  cosdrive(25,62);
  Elevator.spinFor(forward,700,degrees,500,rpm); //sixth goal

  cosdrive(8.5,-35);
  Pturn(-90);
  Intakes.spin(forward,100,percent);
  cosdrive(58,75);
  Intakes.stop();
  Elevator.spinFor(forward,800,degrees,300,rpm,false);
  Pturn(-47);
  cosdrive(11.5,41);
  Elevator.spinFor(forward,1100,degrees,500,rpm); //seventh goal

  cosdrive(46,-75);
  Pturn(-185);
  Intakes.spin(forward,100,percent);
  cosdrive(26,60);
  Intakes.stop();
  Elevator.spinFor(forward,1400,degrees,300,rpm,false);
  Pturn(-272);
  cosdrive(18,60);
  Elevator.spinFor(forward,800,degrees,320,rpm); //eigth goal
  cosdrive(18,-100);
}