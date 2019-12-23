#include <AccelStepper.h>
#include <Servo.h>

#define dirPinLength 6
#define stepPinLength 13

#define dirPinGrip 2
#define stepPinGrip 3
#define dirPinHeight 4
#define stepPinHeight 5

#define gripperEnable 7
#define heightEnable 8
#define widthEnable 9
#define lengthEnable 10

#define dirPinWidth 11
#define stepPinWidth 12

#define motorInterfaceType 1
volatile int servoAngle=90;
volatile int targetServoAngle=0;
AccelStepper gripperStepper = AccelStepper(motorInterfaceType, stepPinGrip, dirPinGrip);
AccelStepper heightStepper = AccelStepper(motorInterfaceType, stepPinHeight, dirPinHeight);
AccelStepper widthStepper = AccelStepper(motorInterfaceType, stepPinWidth, dirPinWidth);
AccelStepper lengthStepper = AccelStepper(motorInterfaceType, stepPinLength, dirPinLength);
Servo myServo;

void setup() {
  pinMode(widthEnable,OUTPUT);
  digitalWrite(widthEnable,HIGH);
  pinMode(lengthEnable,OUTPUT);
  digitalWrite(lengthEnable,HIGH);
  pinMode(gripperEnable,OUTPUT);
  digitalWrite(gripperEnable,LOW);
  pinMode(heightEnable,OUTPUT);
  digitalWrite(heightEnable,HIGH);
  myServo.attach(A0); 
  gripperStepper.setMaxSpeed(300);
  gripperStepper.setAcceleration(300);
  heightStepper.setMaxSpeed(300);
  heightStepper.setAcceleration(300);
  widthStepper.setMaxSpeed(180);
  widthStepper.setAcceleration(300);
  lengthStepper.setMaxSpeed(300);
  lengthStepper.setAcceleration(300);
  Serial.begin(9600);
  resetGripperPosition();
  resetHeight();
  resetWidth();
  resetLength();
  gripperStepper.setCurrentPosition(clawSpacingToSteps(0.7));
  
}

void loop() {
  receiveCommands();
  
}

void driveClaw(){
while (gripperStepper.distanceToGo()!=0 || heightStepper.distanceToGo()!=0){
  driveGripperStepper();
  driveHeightStepper();
  }
}

void resetHeight(){

}
void resetWidth(){
  
}
void resetLength(){

  
}

void receiveCommands(){
 if (Serial.available()>0){
  String tempstr=Serial.readStringUntil(10);
    if (tempstr.indexOf("A")==0){ //Adjust angle
      tempstr.remove(0,1);
      myServo.write(tempstr.toInt()+39);
    }
    if (tempstr.indexOf("G")==0){ //Adjust gripper finger spacing
      tempstr.remove(0,1);
      heightStepper.move(heightAdjustment(gripperStepper.currentPosition(),clawSpacingToSteps(tempstr.toFloat())));
      gripperStepper.setSpeed(300*heightAdjustment(gripperStepper.currentPosition(),clawSpacingToSteps(tempstr.toFloat()))/abs(gripperStepper.currentPosition()-clawSpacingToSteps(tempstr.toFloat())));
      gripperStepper.moveTo(clawSpacingToSteps(tempstr.toFloat()));
      
    }
    if (tempstr.indexOf("H")==0){
      tempstr.remove(0,1);
      heightStepper.move(tempstr.toFloat()/0.4*200.0);//adjust height platform
      digitalWrite(heightEnable,LOW);
      delay(3);
      heightStepper.runToPosition();
      delay(3);
      digitalWrite(heightEnable,HIGH);
    }
    if (tempstr.indexOf("W")==0){//adjust width platform
      tempstr.remove(0,1);
      if (tempstr.toFloat()<0){
      takeSteps(-1*tempstr.toFloat()/0.4*200,LOW,dirPinWidth, stepPinWidth, widthEnable);
      }else{
      takeSteps(tempstr.toFloat()/0.4*200,HIGH,dirPinWidth, stepPinWidth, widthEnable);
      }
    }
    if (tempstr.indexOf("L")==0){//adjust length platform
      tempstr.remove(0,1);
      if (tempstr.toFloat()<0){
      takeSteps(-1.0*tempstr.toFloat()/0.8*200,LOW,dirPinLength, stepPinLength, lengthEnable);
      }else{
      takeSteps(tempstr.toFloat()/0.8*200,HIGH,dirPinLength, stepPinLength, lengthEnable);
      }
    }
    if (tempstr.indexOf("RG")==0){//reinitializes gripper position
      resetGripperPosition();
    }
  }
}

void driveGripperStepper(){
   if (gripperStepper.distanceToGo()!=0){
   digitalWrite(gripperEnable,LOW);
   delay(3);
   gripperStepper.run();
   delay(3);
  }
digitalWrite(gripperEnable,HIGH);//turn off current to motor
}

void driveHeightStepper(){
  if (heightStepper.distanceToGo()!=0){
   digitalWrite(heightEnable,LOW);
   delay(3);
   heightStepper.run();
   delay(3);
  }
  digitalWrite(heightEnable,HIGH);//turn off current to motor
}

void resetGripperPosition(){
  digitalWrite(gripperEnable,LOW);
  delay(5);
  gripperStepper.moveTo(gripperStepper.currentPosition()+200);
  gripperStepper.runToPosition(); //this ensures stepper is reset to fully closed position
  gripperStepper.setCurrentPosition(clawSpacingToSteps(0.7));
  Serial.println("position reset");
   digitalWrite(gripperEnable,HIGH);
}

int clawSpacingToSteps(float spacing){
//This function converts the desired finger spacing into claw stepper position
//it does this by solving the gripper equation using bisection
float dLow=1;
float dHigh=4;
float dAvg=(dLow+dHigh)/2;
  if ((clawFunc(dLow)-spacing)*(clawFunc(dHigh)-spacing)>0){
    //in the case that the target finger spacing is out of range, error is displayed
    Serial.println("Error: gripper target outside range. Please restart code.");
  }

  while (1){
    // bisection to solve gripper function
    if ((clawFunc(dAvg)-spacing)*(clawFunc(dHigh)-spacing)>0){
    dHigh=dAvg;
    }else{
    dLow=dAvg;
    }
    
    dAvg=(dLow+dHigh)/2;
    if (abs(clawFunc(dAvg)-spacing)<0.01){
      break;
    }
  }
    //1 rotation=200steps=0.4cm
return -1.0*dAvg/0.4*200;
}


int heightAdjustment(float initialPos, float finalPos){
  //input is initial and final position for gripper shift
  //returns height stepper position shift to compensate for gripper contraction;
  float dinitial=-initialPos*0.4/200;
  float dfinal=-finalPos*0.4/200;
return -1.0*(heightFunc(dinitial)-heightFunc(dfinal))/0.4*200.0;
}


float clawFunc(float D){
  //evaluate claw contraction function
float L1=2.3;
float L2=2.3;
float L3=2;
float theta2=130*3.14159/180;
float theta1=acos((D*D+L2*L2-L1*L1)/(2*L2*D));
float deltaP=L3*sin(3.14159-theta1-theta2);
return 1.0*(deltaP*2.0+1.9);
}

float heightFunc(float D){
  //evaluate height shift function
float L1=2.3;
float L2=2.3;
float L3=2;
float theta2=130*3.14159/180;
float theta1=acos((D*D+L2*L2-L1*L1)/(2*L2*D));
float deltaH=D+L3*cos(3.14159-theta1-theta2);
return 1.0*(deltaH);
}

void takeSteps(int n,int dir, int dirPin, int stepPin, int enablePin) {
    //Rotate "n" steps
  digitalWrite(dirPin, dir);
  digitalWrite(enablePin,LOW);
  delay(3);
  for(int x = 0; x < n; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  delay(3);
  digitalWrite(enablePin,HIGH);
}
