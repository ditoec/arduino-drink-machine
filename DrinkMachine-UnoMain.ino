
#include <VarSpeedServo.h>
#include <EEPROM.h>
#include <AccelStepper.h>

#define S1_PULSE 2
#define S1_DIR 3
#define S2_PULSE 4
#define S2_DIR 5
#define S3_PULSE 6
#define S3_DIR 7
#define S4_PULSE 8
#define S4_DIR 9
#define S5_PULSE 10
#define S5_DIR 11
#define S6_PULSE 12
#define S6_DIR 13
#define STEPS_PER_REV 200
#define PT_IN 0
#define LS_IN 1
#define FSV A2
#define PRS_OUT A3
#define PIS_OUT A4
#define DS_OUT A5

#define OPEN 0
#define CLOSE 180

#define FAST 255
#define SLOW 55

#define PT_LOW 102
#define PT_RANGE 818
#define PT_MAX 120

#define LS_HIGH 900
#define LS_MED 600
#define LS_LOW 300

// Define some steppers and the pins the will be used
AccelStepper S1(AccelStepper::DRIVER,S1_PULSE,S1_DIR);
AccelStepper S2(AccelStepper::DRIVER,S2_PULSE,S2_DIR);
AccelStepper S3(AccelStepper::DRIVER,S3_PULSE,S3_DIR);
AccelStepper S4(AccelStepper::DRIVER,S4_PULSE,S4_DIR);
AccelStepper S5(AccelStepper::DRIVER,S5_PULSE,S5_DIR);
AccelStepper S6(AccelStepper::DRIVER,S6_PULSE,S6_DIR);
// create servo object to control a servo
VarSpeedServo PRS;
VarSpeedServo PIS;
VarSpeedServo DS;
// create string variable for Serial input
String comm;

void setup() {
  //setup IO mode
  pinMode(FSV, OUTPUT);
  pinMode(PRS_OUT,OUTPUT);
  pinMode(PIS_OUT,OUTPUT);
  pinMode(DS_OUT,OUTPUT);
  //setup stepper
  S1.setMaxSpeed(STEPS_PER_REV);
  S1.setAcceleration(4*STEPS_PER_REV);
  S2.setMaxSpeed(STEPS_PER_REV);
  S2.setAcceleration(4*STEPS_PER_REV);
  S3.setMaxSpeed(STEPS_PER_REV);
  S3.setAcceleration(4*STEPS_PER_REV);
  S4.setMaxSpeed(STEPS_PER_REV);
  S4.setAcceleration(4*STEPS_PER_REV);
  S5.setMaxSpeed(STEPS_PER_REV);
  S5.setAcceleration(4*STEPS_PER_REV);
  S6.setMaxSpeed(STEPS_PER_REV);
  S6.setAcceleration(4*STEPS_PER_REV);
  //attach servo pin to servo object
  PRS.attach(PRS_OUT);
  PIS.attach(PIS_OUT);
  DS.attach(DS_OUT);
  //set default state
  PRS.write(OPEN,FAST);
  PIS.write(CLOSE,FAST);
  DS.write(CLOSE,FAST);
  digitalWrite(FSV,LOW);
  
  EEPROM.write(0,60);
  EEPROM.write(1,2);
  ///
  EEPROM.write(2,10);
  EEPROM.write(3,10);
  EEPROM.write(4,10);
  EEPROM.write(5,10);
  EEPROM.write(6,10);
  EEPROM.write(7,10);
  EEPROM.write(8,1);
  //
  EEPROM.write(9,20);
  EEPROM.write(10,20);
  EEPROM.write(11,20);
  EEPROM.write(12,20);
  EEPROM.write(13,20);
  EEPROM.write(14,20);
  EEPROM.write(15,20);
  //start serial comm
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()>0){
    comm = Serial.readString();
    int numberOfPieces = 0;
    if(comm.charAt(0)=='M'){
      numberOfPieces = 3;
    }
    else if(comm.charAt(0)=='D'){
      numberOfPieces = 9;
    }
    String pieces[numberOfPieces];
    int counter = 0;
    int lastIndex = 0;
    for (int i = 0; i < comm.length(); i++) {
      // Loop through each character and check if it's a comma
      if (comm.substring(i, i+1) == ",") {
      	// Grab the piece from the last index up to the current position and store it
      	pieces[counter] = comm.substring(lastIndex, i);
      	// Update the last position and add 1, so it starts from the next character
      	lastIndex = i + 1;
      	// Increase the position in the array that we store into
      	counter++;
      }
      // If we're at the end of the string (no more commas to stop us)
      if (i == comm.length() - 1) {
        // Grab the last part of the string from the lastIndex to the end
        pieces[counter] = comm.substring(lastIndex, i);
      }
    }
    if(comm.charAt(0)=='M'){
      moveStepper(pieces[1].toInt(),pieces[2].toFloat());
    }
    else if(comm.charAt(0)=='D'){
      dispense(pieces[1].toInt(),pieces[2].toInt(),pieces[3].toInt(),pieces[4].toInt(),pieces[5].toInt(),pieces[6].toInt(),pieces[7].toInt(),pieces[8].toInt());
    }
    if (Serial.available()>0)comm = Serial.readString();
  }
  if(analogRead(PT_IN)>150&&analogRead(PT_IN)<250){
    delay(50);
    if(analogRead(PT_IN)>150&&analogRead(PT_IN)<250){
      dispenseEEPROM(1);
      if (Serial.available()>0)comm = Serial.readString();
    }
  }  
}

void moveStepper(int stepperNum, float rotation){
  switch(stepperNum){
    case 0:
      //turn all the steppers
      S1.move(rotation*STEPS_PER_REV);
      S2.move(rotation*STEPS_PER_REV);
      S3.move(rotation*STEPS_PER_REV);
      S4.move(rotation*STEPS_PER_REV);
      S5.move(rotation*STEPS_PER_REV);
      S6.move(rotation*STEPS_PER_REV);
      while(S1.distanceToGo()>0||S2.distanceToGo()>0||S3.distanceToGo()>0||S4.distanceToGo()>0||S5.distanceToGo()>0||S6.distanceToGo()>0)
      {
        S1.run();S2.run();S3.run();
        S4.run();S5.run();S6.run();
      }
      break;
    case 1:
      //turn stepper 1
      S1.move(rotation*STEPS_PER_REV);
      while(S1.distanceToGo()>0)S1.run();
      break;
    case 2:
      //turn stepper 2
      S2.move(rotation*STEPS_PER_REV);
      while(S2.distanceToGo()>0)S2.run();
      break;
    case 3:
      //turn stepper 3
      S3.move(rotation*STEPS_PER_REV);
      while(S3.distanceToGo()>0)S3.run();
      break;
    case 4:
      //turn stepper 4
      S4.move(rotation*STEPS_PER_REV);
      while(S4.distanceToGo()>0)S4.run();
      break;
    case 5:
      //turn stepper 5
      S5.move(rotation*STEPS_PER_REV);
      while(S5.distanceToGo()>0)S5.run();
      break;
    case 6:
      //turn stepper 6
      S6.move(rotation*STEPS_PER_REV);
      while(S6.distanceToGo()>0)S6.run();
      break;
  }
}

void dispense(int pressure,int S1_turns,int S2_turns,int S3_turns,int S4_turns,int S5_turns,int S6_turns,int water){
  //start sequence
  if(water){
    //FSV open
    digitalWrite(FSV,HIGH);
    //wait for LS LOW, MED, HIGH
    while(analogRead(LS_IN)<LS_LOW)delay(1);
    while(analogRead(LS_IN)<LS_MED)delay(1);
    while(analogRead(LS_IN)<LS_HIGH)delay(1);
    //then close FSV+PRS
    digitalWrite(FSV,LOW);
    PRS.write(CLOSE,FAST);
    //PIS slowly opens
    PIS.write(OPEN,SLOW);
    //wait until PT reach required level
    while(analogRead(PT_IN)<pressure)delay(1);
    //PIS close
    PIS.write(CLOSE,FAST);
    //wait 2 second
    delay(2000);
    //PRS slowly opens
    PRS.write(OPEN,SLOW);
    //wait until PT reach 0
    while(analogRead(PT_IN)>PT_LOW)delay(1);
  }
  //DS opens
  DS.write(OPEN,FAST);
  //turn all the steppers
  S1.move(S1_turns*STEPS_PER_REV);
  S2.move(S2_turns*STEPS_PER_REV);
  S3.move(S3_turns*STEPS_PER_REV);
  S4.move(S4_turns*STEPS_PER_REV);
  S5.move(S5_turns*STEPS_PER_REV);
  S6.move(S6_turns*STEPS_PER_REV);
  while(S1.distanceToGo()>0||S2.distanceToGo()>0||S3.distanceToGo()>0||S4.distanceToGo()>0||S5.distanceToGo()>0||S6.distanceToGo()>0)
  {
    S1.run();S2.run();S3.run();
    S4.run();S5.run();S6.run();
  }
  //wait for LS LOW
  while(analogRead(LS_IN)>LS_LOW)delay(1);
  //wait for 3 secs
  delay(3000);
  //close DS
  DS.write(CLOSE,FAST);
}

void dispenseEEPROM(int mix){
  //get required data from EEPROM
  int pressure = PT_LOW+((EEPROM.read(0)*PT_RANGE)/PT_MAX);
  int S1_turns = EEPROM.read(2+(mix*7));
  int S2_turns = EEPROM.read(2+(mix*7)+1);
  int S3_turns = EEPROM.read(2+(mix*7)+2);
  int S4_turns = EEPROM.read(2+(mix*7)+3);
  int S5_turns = EEPROM.read(2+(mix*7)+4);
  int S6_turns = EEPROM.read(2+(mix*7)+5);
  int water    = EEPROM.read(2+(mix*7)+6);
  //start sequence
  if(water){
    //FSV open
    digitalWrite(FSV,HIGH);
    //wait for LS LOW, MED, HIGH
    while(analogRead(LS_IN)<LS_LOW)delay(1);
    while(analogRead(LS_IN)<LS_MED)delay(1);
    while(analogRead(LS_IN)<LS_HIGH)delay(1);
    //then close FSV+PRS
    digitalWrite(FSV,LOW);
    PRS.write(CLOSE,FAST);
    //PIS slowly opens
    PIS.write(OPEN,SLOW);
    //wait until PT reach required level
    while(analogRead(PT_IN)<pressure)delay(1);
    //PIS close
    PIS.write(CLOSE,FAST);
    //wait 2 second
    delay(2000);
    //PRS slowly opens
    PRS.write(OPEN,SLOW);
    //wait until PT reach 0
    while(analogRead(PT_IN)>PT_LOW)delay(1);
  }
  //DS opens
  DS.write(OPEN,FAST);
  //turn all the steppers
  S1.move(S1_turns*STEPS_PER_REV);
  S2.move(S2_turns*STEPS_PER_REV);
  S3.move(S3_turns*STEPS_PER_REV);
  S4.move(S4_turns*STEPS_PER_REV);
  S5.move(S5_turns*STEPS_PER_REV);
  S6.move(S6_turns*STEPS_PER_REV);
  while(S1.distanceToGo()>0||S2.distanceToGo()>0||S3.distanceToGo()>0||S4.distanceToGo()>0||S5.distanceToGo()>0||S6.distanceToGo()>0)
  {
    S1.run();S2.run();S3.run();
    S4.run();S5.run();S6.run();
  }
  //wait for LS LOW
  while(analogRead(LS_IN)>LS_LOW)delay(1);
  //wait for 3 secs
  delay(3000);
  //close DS
  DS.write(CLOSE,FAST);
}
