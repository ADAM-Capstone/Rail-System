#include <AccelStepper.h>
#include <Servo.h>
#include <ArduinoNunchuk.h>

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

#define upThresholdY 190
#define upThresholdX 190
#define downThresholdY 60
#define downThresholdX 60

#define motorInterfaceType 1
volatile int servoAngle=90;
volatile int targetServoAngle=0;
AccelStepper gripperStepper = AccelStepper(motorInterfaceType, stepPinGrip, dirPinGrip);
AccelStepper heightStepper = AccelStepper(motorInterfaceType, stepPinHeight, dirPinHeight);
AccelStepper widthStepper = AccelStepper(motorInterfaceType, stepPinWidth, dirPinWidth);
AccelStepper lengthStepper = AccelStepper(motorInterfaceType, stepPinLength, dirPinLength);
Servo myServo;
ArduinoNunchuk nunchuk = ArduinoNunchuk();
// SCL is connected to A5
// SDA is connected to A4
// +3.3V connected to +
// GND connected to -
int X_value = 125;
int Y_value = 132;
int c_button=0;
int z_button=0;

int stepInterval=10;
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
  nunchuk.init();
  gripperStepper.setMaxSpeed(3000);
  gripperStepper.setAcceleration(3000);
  heightStepper.setMaxSpeed(3000);
  heightStepper.setAcceleration(3000);
  widthStepper.setMaxSpeed(180);
  widthStepper.setAcceleration(300);
  lengthStepper.setMaxSpeed(300);
  lengthStepper.setAcceleration(300);
  Serial.begin(9600);
  gripperStepper.setCurrentPosition(clawSpacingToSteps(0.7));
}

void loop() {
  receiveCommands();
  nunchukControl();
  driveGripperStepper();
  driveHeightStepper();
  driveAngle();
}

void nunchukControl(){
  nunchuk.update();
  X_value = nunchuk.analogX;
  Y_value = nunchuk.analogY;
  c_button=nunchuk.cButton;
  z_button=nunchuk.zButton;
  
    if (X_value <= downThresholdX && z_button==0 && c_button==0) { //Joystick left

    digitalWrite(lengthEnable,LOW);
    digitalWrite(dirPinLength,HIGH);
    delay(3);
    while(1){
      for (int i=0; i<stepInterval;i++){
    digitalWrite(stepPinLength, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinLength, LOW);
    delayMicroseconds(500);}
      nunchuk.update();
      X_value = nunchuk.analogX;
      Y_value = nunchuk.analogY;
      c_button=nunchuk.cButton;
      z_button=nunchuk.zButton;
      if (X_value>=downThresholdX)
        break;
    }
    delay(3);
    digitalWrite(lengthEnable,HIGH);//turn off current to motor


  } 
  if (X_value >= upThresholdX && z_button==0 && c_button==0) { //Joystick right
    digitalWrite(lengthEnable,LOW);
    delay(3);
    digitalWrite(dirPinLength,LOW);
    while(1){
      for (int i=0; i<stepInterval;i++){
    digitalWrite(stepPinLength, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinLength, LOW);
    delayMicroseconds(500);}
      nunchuk.update();
      X_value = nunchuk.analogX;
      Y_value = nunchuk.analogY;
      c_button=nunchuk.cButton;
      z_button=nunchuk.zButton;
      if (X_value<=upThresholdX)
        break;
    }
    delay(3);
    digitalWrite(lengthEnable,HIGH);//turn off current to motor
    Serial.println("LENGTH RIGHT: ");

  }

  if (Y_value <= downThresholdY && z_button==0 && c_button==0) { //Joystick down
    digitalWrite(widthEnable,LOW);
    delay(3);
    digitalWrite(dirPinWidth,HIGH);
    while(1){
    for (int i=0; i<stepInterval;i++){
    digitalWrite(stepPinWidth, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinWidth, LOW);
    delayMicroseconds(500);}
      nunchuk.update();
      X_value = nunchuk.analogX;
      Y_value = nunchuk.analogY;
      c_button=nunchuk.cButton;
      z_button=nunchuk.zButton;
      if (X_value>=downThresholdY)
        break;
    }
    delay(3);
    digitalWrite(widthEnable,HIGH);//turn off current to motor

  }

  if (Y_value >= upThresholdY && z_button==0 && c_button==0) { //Joystick up
    digitalWrite(widthEnable,LOW);
    delay(3);
    digitalWrite(dirPinWidth,LOW);
    while(1){
    for (int i=0; i<stepInterval;i++){
    digitalWrite(stepPinWidth, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinWidth, LOW);
    delayMicroseconds(500);}
      nunchuk.update();
      X_value = nunchuk.analogX;
      Y_value = nunchuk.analogY;
      c_button=nunchuk.cButton;
      z_button=nunchuk.zButton;
      if (X_value<=upThresholdY)
        break;
    }
    delay(3);
    digitalWrite(widthEnable,HIGH);//turn off current to motor

  }

  
  //This is other modes for height and claw contraction
  if (Y_value <= downThresholdY && z_button==1 && c_button==0) { //Joystick down
    digitalWrite(heightEnable,LOW);
    delay(3);
    heightStepper.move(heightAdjustment(gripperStepper.currentPosition(),gripperStepper.currentPosition()+15));
    heightStepper.runToPosition();
    delay(3);
    digitalWrite(heightEnable,HIGH);
    
    digitalWrite(gripperEnable,LOW);
    delay(3);
    gripperStepper.move(15);
    gripperStepper.runToPosition();
    delay(3);
    digitalWrite(gripperEnable,HIGH);//turn off current to motor
    Serial.println("CLAW CONTRACTING");
  }

  if (Y_value >= upThresholdY && z_button==1 && c_button==0) { //Joystick up

    digitalWrite(heightEnable,LOW);
    delay(3);
    heightStepper.move(heightAdjustment(gripperStepper.currentPosition(),gripperStepper.currentPosition()-15));
    heightStepper.runToPosition();
    delay(3);
    digitalWrite(heightEnable,HIGH);
    
    digitalWrite(gripperEnable,LOW);
    delay(3);
    gripperStepper.move(-15);
    gripperStepper.runToPosition();
    delay(3);
    digitalWrite(gripperEnable,HIGH);//turn off current to motor
    Serial.println("CLAW LOOSENING ");
  }
  if (Y_value <= downThresholdY && z_button==0 && c_button==1) { //Joystick down
    digitalWrite(heightEnable,LOW);
    delay(3);
    digitalWrite(dirPinHeight,LOW);
    while(1){
    for (int i=0; i<stepInterval;i++){
    digitalWrite(stepPinHeight, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinHeight, LOW);
    delayMicroseconds(500);}
      nunchuk.update();
      X_value = nunchuk.analogX;
      Y_value = nunchuk.analogY;
      c_button=nunchuk.cButton;
      z_button=nunchuk.zButton;
      if (X_value>=downThresholdY)
        break;
    }
    delay(3);
    digitalWrite(gripperEnable,HIGH);//turn off current to motor
  }

  if (Y_value >= upThresholdY && z_button==0 && c_button==1) { //Joystick up
    digitalWrite(heightEnable,LOW);
    delay(3);
    digitalWrite(dirPinHeight,HIGH);
    while(1){
      for (int i=0; i<stepInterval;i++){
    digitalWrite(stepPinHeight, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinHeight, LOW);
    delayMicroseconds(500);}
      nunchuk.update();
      X_value = nunchuk.analogX;
      Y_value = nunchuk.analogY;
      c_button=nunchuk.cButton;
      z_button=nunchuk.zButton;
      if (X_value<=upThresholdY)
        break;
    }
    delay(3);
    digitalWrite(gripperEnable,HIGH);//turn off current to motor

  }
  
X_value = 125;
Y_value = 132;
c_button=0;
z_button=0;

}

void receiveCommands(){
 if (Serial.available()>0){
  String tempstr=Serial.readStringUntil(10);
    if (tempstr.indexOf("ANGLE")==0){
      tempstr.remove(0,5);
      targetServoAngle=tempstr.toInt();
    }
    if (tempstr.indexOf("GRIPPER")==0){
      tempstr.remove(0,7);
      heightStepper.move(heightAdjustment(gripperStepper.currentPosition(),clawSpacingToSteps(tempstr.toFloat())));
      gripperStepper.moveTo(clawSpacingToSteps(tempstr.toFloat()));
    }
    if (tempstr.indexOf("HEIGHT")==0){
      tempstr.remove(0,6);
      heightStepper.move(tempstr.toInt());
    }
    if (tempstr.indexOf("WIDTH")==0){
      tempstr.remove(0,5);
      if (tempstr.toInt()<0){
      takeSteps(-1*tempstr.toInt(),LOW,dirPinWidth, stepPinWidth, widthEnable);
      }else{
      takeSteps(tempstr.toInt(),HIGH,dirPinWidth, stepPinWidth, widthEnable);
      }
    }
    if (tempstr.indexOf("LENGTH")==0){
      tempstr.remove(0,6);
      if (tempstr.toInt()<0){
      takeSteps(-1*tempstr.toInt(),LOW,dirPinLength, stepPinLength, lengthEnable);
      }else{
      takeSteps(tempstr.toInt(),HIGH,dirPinLength, stepPinLength, lengthEnable);
      }
    }
    if (tempstr.indexOf("GRIPACCEL")==0){
      tempstr.remove(0,9);
      gripperStepper.setAcceleration(tempstr.toFloat());
    }
    if (tempstr.indexOf("GRIPSPEED")==0){
      tempstr.remove(0,9);
      gripperStepper.setMaxSpeed(tempstr.toFloat());
    } 
    if (tempstr.indexOf("RESETGRIPPER")==0){
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

void driveAngle(){
  if (targetServoAngle!=servoAngle){
    myServo.write(targetServoAngle+39);
    servoAngle=targetServoAngle;
    }
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
  //input is in cm
// MIN CLAW SPACING IS 0.9CM -> MEANS lowest D=2cm
float dLow=1;
float dHigh=4;
float dAvg=(dLow+dHigh)/2;
  if ((clawFunc(dLow)-spacing)*(clawFunc(dHigh)-spacing)>0){
    Serial.println("WRONG INITIAL GUESS");
  }

    
  while (1){
    // bisection to solve function
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

void goToCoord(float Length, float Width, float Height){
  
}
