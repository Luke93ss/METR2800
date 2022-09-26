#include <Stepper.h>
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(2, 3); // RX, TX
#define directionpin1 12 // Direction of stepper1
#define step1 13 // Pwm input for stepper3 
#define directionpin2 10 // Direction of stepper1
#define step2 11 // Pwm input for stepper2
#define directionpin3 6 // Direction of stepper1
#define step3 5 // Pwm input for stepper3

const int STEPS_PER_REV = 200;


void setup() {
  // put your setup code here, to run once:

   // Debug console
  DebugSerial.begin(9600);

  DebugSerial.println("Waiting for connections...");

 
  Serial.begin(9600);
  
  pinMode(directionpin1,OUTPUT); // Sets directionpin1 as output
  pinMode(directionpin2,OUTPUT); // Sets directionpin2 as output
  pinMode(directionpin3,OUTPUT); // Sets directionpin3 as output
  pinMode(step1,OUTPUT); // Sets step1 as output
  pinMode(step2,OUTPUT); // Sets step2 as output
  pinMode(step3,OUTPUT); // Sets step3 as output

}


void setdirection(int dir, int dirpin){

  if (dir == 0){
    digitalWrite(dirpin, LOW);
  }
  if (dir == 1){
    digitalWrite(dirpin, HIGH);
  }
}
// Use this runMotor function for a specified distance.
void runMotor(int rotations){
    
    for (int x = 0; x < STEPS_PER_REV*rotations; x++){
      digitalWrite(step3,HIGH);
      delayMicroseconds(700); // Minimum delay is approx 350 uS
      digitalWrite(step3,LOW);
      delayMicroseconds(700); // Minimum delay is approx 350 uS
    }
  
}

void runMotor2(some sensor input (TOF sensor?){

    while( something true)
      digitalWrite(step3,HIGH);
      delayMicroseconds(700); // Minimum delay is approx 350 uS
      digitalWrite(step3,LOW);
      delayMicroseconds(700); // Minimum delay is approx 350 uS
      }

void loop() {

  delayMicroseconds(2000);
  setdirection(0,directionpin3);
  runMotor(20);
  delay(1000);
  setdirection(1,directionpin3);
  runMotor(5);
  delay(5000);

  

}
