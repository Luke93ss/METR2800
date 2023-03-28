#include <VL53L0X.h>
#include <Wire.h>

//#include "Adafruit_VL53L0X.h"
#include <Servo.h>

// Macros to define all possible states. 
#define WAIT_STATE 0 
#define LOCATE_ZONE_C 1 
#define PICK_UP 2
#define PUT_DOWN 3
#define RETRACT_ARM 4
#define RAISE_BODY 5
#define OUTBOUND_CABLE_TRANSLATE 6
#define INBOUND_CABLE_TRANSLATE 16
#define ORIENTATE_BODY_ZONE_C 7
#define LOWER_BODY 8
#define COMPLETE 9
#define RAISE_CLAW 10
#define LOWER_CLAW 11
#define ZONE_C_ROTATE_RIGHT 12
#define ZONE_C_ROTATE_LEFT 13
#define LOCATE_PACKAGE 14
#define ZONE_C_TRANSLATE_RIGHT 15
#define DROP_PACKAGE 17
#define TESTING 18
#define ORIENTATE_BODY_HOME 19

// Global variables used as control inputs 
// for state transitions, and to track current state.

// Interrupts need to be coded to detect changes and adjust the buttonState, hasFoundZoneC, and isRaised variables
// /\ replaced with a check function in void loop
// Buttons
#define RIGHT_INNER_LIMIT 0
#define RIGHT_OUTER_LIMIT 1
#define LEFT_INNER_LIMIT 2
#define LEFT_OUTER_LIMIT 3
#define START_BUTTON 4
#define RAISE_LOWER_TOP_LIMIT 5
#define RAISE_LOWER_BOTTOM_LIMIT 6
#define CLAW_TOP_LIMIT 7
#define CLAW_LOWER_LIMIT 8
#define ROTATE_START_LIMIT 9
#define ROTATE_END_LIMIT 10
// Constants for Buttons
#define NUMBER_OF_BUTTONS 11
#define BUTTON_TIME 40

uint8_t buttons_pins[] = {43, 44, 45, 46, 49, 50, 51, 52, 53, 41, 42};
unsigned long button_times[NUMBER_OF_BUTTONS]; // time of last button press
bool button_states[NUMBER_OF_BUTTONS];


uint16_t currentState = 0;
int buttonState = 0; 
boolean hasPackage = false;
boolean hasFoundZoneC = false;
boolean isRaised = false;
boolean hasTranslated = false;
boolean translating = true;
boolean started_locating_zone_C = false;

unsigned long last_tof_time;
#define TOF_DELAY 550

double startBearing; 

// Claw Mechanism Servos

int clawservopin = 3; //Blue servo initialisation
Servo clawservo;

#define CLAW_OPEN_ANGLE 150
#define CLAW_CLOSED_ANGLE 180
#define CLAW_RAISED_ANGLE 180
#define CLAW_LOWERED_ANGLE 0

// DC Motors

#define OMNI_RIGHT 0
#define OMNI_LEFT 1
#define CABLE_PULLEY_MOTOR 2
#define LEAD_SCREW_MOTOR 3
#define SWIVEL_MOTOR 4
#define CLAW_MOTOR 5

#define MOTOR_ENABLE 0
#define MOTOR_IN1 1
#define MOTOR_IN2 2
#define NUMBER_OF_MOTORS 6

int MOTORS[][3] = { // EN, IN1, IN2
  {11, 34, 35},
  {6, 36, 37},
  {7, 32, 33},
  {8, 30, 31},
  {9, 28, 29},
  {10, 26, 27}
};

uint16_t MOTOR_SCALES[NUMBER_OF_MOTORS] = {255, 200, 255, 255, 255, 255};

// Directions
#define OMNI_FORWARDS 0
#define OMNI_BACKWARDS 1
#define PULLEY_OUTBOUND 0
#define PULLEY_INBOUND 1
#define LEAD_SCREW_DOWN 1
#define LEAD_SCREW_UP 0
#define CLAW_UP 1
#define CLAW_DOWN 0
#define SWIVEL_END 1
#define SWIVEL_STATE 0

#define SWIVEL_MOTOR_ENCODER_A 18 // This pin needs to have an interrupt
#define SWIVEL_MOTOR_ENCODER_B 25

#define OMNI_LEFT_STRENGTH 255
#define OMNI_RIGHT_STRENGTH 255

volatile int16_t swivel_position = 0;
unsigned long orientate_previous_time;
uint16_t prev_swivel_error;
float prev_error_integral;
float swivel_target; // bearing in degrees



unsigned long rotate_start_time;
// Timing variables / constants
unsigned long zone_C_start_time;
#define DEPOSIT_TIMEOUT 20000 // milliseconds for the deposit phase to last
                              // before giving up and dropping the package

// unsigned long retraction_start_time;


// ToF Sensor

#define SHT_LOX1 22 //ToF sensor initialistion
#define SHT_LOX2 24
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

//Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
//Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
//VL53L0X_RangingMeasurementData_t lox1measure;
//VL53L0X_RangingMeasurementData_t lox2measure;
VL53L0X sensor;

uint16_t last_depth = 0; // previous ToF measurement
int32_t depth_difference;
uint16_t value;
#define TOF_DELTA 30 // Depth for the ToF sensor to know it's passed the chasm

void addState(int state) {
  currentState |= (1 << state);
}

void removeState(int state) {
  currentState |= ~(1 << state);
}

bool hasState(int state) {
  return currentState && (1 << state);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Hello World!");
  Wire.begin();
  clawservo.attach(clawservopin);
  clawservo.write(CLAW_CLOSED_ANGLE); //claw closed

  //blackservo.write(0); //claw lowered
  // Initialise Motor Pins
  for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
      for (int j = 0; j < 3; j++) {
          pinMode(MOTORS[i][j], OUTPUT);
      }
  }

  // Initialise Button Pins

  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
      pinMode(buttons_pins[i], INPUT_PULLUP);
  }
  delay(2000);
  
  /*if (!lox1.begin(0x29, true)) {
      Serial.println(F("Failed to boot VL53L0X"));
      while (true) {}
  }
  */

  // Setup starting position.

  last_tof_time = millis();
  currentState = ORIENTATE_BODY_ZONE_C;
  delay(1000); // delay to get rid of weird start up effects with buttons
}

void loop() {
  // This loop calls the relevant functions to 
  // execute for the current state. 
  // It then updates the control variables
  // and the current state. Eg: 

  
  checkButtons();
  //Serial.println(currentState);
  
  switch (currentState) {

    case WAIT_STATE:
      Serial.println("Waitstate");
      commandHBridge(LEAD_SCREW_MOTOR, 255, LEAD_SCREW_DOWN);
      while (!button_states[RAISE_LOWER_BOTTOM_LIMIT]) {
        checkButtons();
      }
      commandHBridge(LEAD_SCREW_MOTOR, 0, LEAD_SCREW_DOWN);
      
      if (button_states[RIGHT_OUTER_LIMIT]) {
        /*Serial.println("button pressed");
        clawservo.write(CLAW_OPEN_ANGLE);
        delay(2000);
        clawservo.write(CLAW_CLOSED_ANGLE);
        delay(1000);*/
        currentState = LOCATE_PACKAGE;
      }
      break; 

    case LOCATE_PACKAGE:
      commandHBridge(OMNI_LEFT, 255, OMNI_FORWARDS);
      commandHBridge(OMNI_RIGHT, 255, OMNI_FORWARDS);
      openClaw();
      delay(3000);
      commandHBridge(OMNI_LEFT, 0, OMNI_FORWARDS);
      commandHBridge(OMNI_RIGHT, 0, OMNI_FORWARDS);
      closeClaw();
      delay(500);
      currentState = RAISE_CLAW;
      break;

    case RAISE_CLAW:
      Serial.println("RAISE_CLAW");
      commandHBridge(CLAW_MOTOR, 255, CLAW_UP);
      Serial.println("Claw going up");
      if (button_states[CLAW_TOP_LIMIT]) {
        commandHBridge(CLAW_MOTOR, 0, CLAW_UP);
        currentState = RETRACT_ARM;
      }
      break;

    case RETRACT_ARM :
      Serial.println("Inside RETRACT_ARM case");
      commandHBridge(OMNI_LEFT, 255, OMNI_BACKWARDS);
      commandHBridge(OMNI_RIGHT, 255, OMNI_BACKWARDS);
      if (hasPackage) {
        delay(3500);  
      } else {
        delay(10000);
      }
      commandHBridge(OMNI_LEFT, 0, OMNI_FORWARDS);
      commandHBridge(OMNI_RIGHT, 0, OMNI_FORWARDS);     
      currentState = RAISE_BODY;
      /*
      if (hasTranslated && hasPackage) {
        currentState = ORIENTATE_BODY_ZONE_C;
      } else if ((hasTranslated && !hasPackage) || (!hasTranslated && hasPackage)) {
        currentState = RAISE_BODY;
      }
      */
      break;

    case RAISE_BODY:
      if (!hasTranslated) {
        commandHBridge(LEAD_SCREW_MOTOR, 255, LEAD_SCREW_UP);
        Serial.println("Raising Body");
        Serial.println(button_states[RAISE_LOWER_TOP_LIMIT]);
        while (button_states[RAISE_LOWER_TOP_LIMIT]) { //inverse because of whack button
          checkButtons();
          Serial.println(button_states[RAISE_LOWER_TOP_LIMIT]);
          //currentState = hasFoundZoneC ? COMPLETE : OUTBOUND_CABLE_TRANSLATE;
        }
        commandHBridge(LEAD_SCREW_MOTOR, 0, LEAD_SCREW_UP);
        delay(100);
        currentState = OUTBOUND_CABLE_TRANSLATE;
      }
      else if (hasTranslated) {
        commandHBridge(LEAD_SCREW_MOTOR, 255, LEAD_SCREW_UP);
        Serial.println("Raising Body");
        Serial.println(button_states[RAISE_LOWER_BOTTOM_LIMIT]);
        while (button_states[RAISE_LOWER_TOP_LIMIT]) {
          checkButtons();
          //currentState = hasFoundZoneC ? COMPLETE : OUTBOUND_CABLE_TRANSLATE;
        }
        commandHBridge(LEAD_SCREW_MOTOR, 0, LEAD_SCREW_UP);
        currentState = ORIENTATE_BODY_HOME;
        translating = true;
        
      }
       
      
      /*
      Serial.println("Inside RAISE_BODY case");
      raiseBody();
      delay(100);
      currentState = LOWER_BODY; //OUTBOUND CABLE TRANSLATE
      Serial.println("Finished RAISE_BODY case");
      */
      break;

    case OUTBOUND_CABLE_TRANSLATE :
      {
      
        // state covers translation in both directions
        // Runs cable motor until tof sensor detects step change is within certain range
                
        /*
        if (translating) {
          Serial.println("OUTBOUND_CABLE_TRAVERSAL");
          commandHBridge(CABLE_PULLEY_MOTOR, 100, PULLEY_OUTBOUND);
          delay(300);
          commandHBridge(CABLE_PULLEY_MOTOR, 255, PULLEY_OUTBOUND);
          //delay(4000); // to allow time to be over chasm so tof won't be tricked
          last_depth = readToF();
          last_tof_time = millis();
          delay(25);
          translating = false;
        } */
        

        if (last_depth == 0) {
          
          do {
              Serial.println("Initialising Sensor inside cable translate");
              delay(1000);
              sensor.setTimeout(500);
              if (!sensor.init())
              {
                Serial.println("Failed to detect and initialize sensor!");
                continue;
              }
              delay(50);
              last_depth = readToF();
              last_tof_time = millis();
          } while (last_depth > 500);
          commandHBridge(CABLE_PULLEY_MOTOR, 100, PULLEY_OUTBOUND);
          delay(300);
          commandHBridge(CABLE_PULLEY_MOTOR, 255, PULLEY_OUTBOUND);
        }
        
        if ((millis() - last_tof_time) > TOF_DELAY) {
          
          uint16_t current_depth = readToF();
          
          depth_difference = (((int32_t) last_depth - (int32_t) current_depth));
          if (depth_difference < TOF_DELTA || depth_difference > 10000) {
            Serial.print("Distance (mm): "); Serial.println(current_depth);
            Serial.print("Difference: "); Serial.println(depth_difference);
            last_depth = current_depth;
            last_tof_time = millis();
          } else { //discard garbage
            // we have found the other side!!
            Serial.println("Distance triggered!");
            Serial.print("Final Distance: "); Serial.println(depth_difference);
            hasTranslated = true;
            translating = true;
            delay(1000);
            commandHBridge(CABLE_PULLEY_MOTOR, 0, PULLEY_OUTBOUND);
            currentState = ORIENTATE_BODY_ZONE_C;
            last_depth = 0;
            Wire.end();
          }
        }
        break;
      }

    case INBOUND_CABLE_TRANSLATE :
      {
      
        // state covers translation in both directions
        // Runs cable motor until tof sensor detects step change is within certain range
                
        /*
        if (translating) {
          Serial.println("OUTBOUND_CABLE_TRAVERSAL");
          commandHBridge(CABLE_PULLEY_MOTOR, 100, PULLEY_OUTBOUND);
          delay(300);
          commandHBridge(CABLE_PULLEY_MOTOR, 255, PULLEY_OUTBOUND);
          //delay(4000); // to allow time to be over chasm so tof won't be tricked
          last_depth = readToF();
          last_tof_time = millis();
          delay(25);
          translating = false;
        } */
        

        if (last_depth == 0) {
          
          do {
              Serial.println("Initialising Sensor inside cable translate");
              delay(1000);
              sensor.setTimeout(500);
              if (!sensor.init())
              {
                Serial.println("Failed to detect and initialize sensor!");
                continue;
              }
              delay(50);
              last_depth = readToF();
              last_tof_time = millis();
          } while (last_depth > 500);
          commandHBridge(CABLE_PULLEY_MOTOR, 100, PULLEY_INBOUND);
          delay(300);
          commandHBridge(CABLE_PULLEY_MOTOR, 255, PULLEY_INBOUND);
        }
        
        if ((millis() - last_tof_time) > TOF_DELAY) {
          
          uint16_t current_depth = readToF();
          
          depth_difference = (((int32_t) last_depth - (int32_t) current_depth));
          if (depth_difference < TOF_DELTA || depth_difference > 10000) {
            Serial.print("Distance (mm): "); Serial.println(current_depth);
            Serial.print("Difference: "); Serial.println(depth_difference);
            last_depth = current_depth;
            last_tof_time = millis();
          } else { //discard garbage
            // we have found the other side!!
            Serial.println("Distance triggered!");
            Serial.print("Final Distance: "); Serial.println(depth_difference);
            hasTranslated = true;
            translating = true;
            delay(1500);
            commandHBridge(CABLE_PULLEY_MOTOR, 0, PULLEY_OUTBOUND);
            currentState = ORIENTATE_BODY_ZONE_C;
            last_depth = 0;
            Wire.end();
          }
        }
        break;
      }

    case LOWER_BODY :
      Serial.println("LOWER_BODY");
      commandHBridge(LEAD_SCREW_MOTOR, 255, LEAD_SCREW_DOWN);
      Serial.println("Lowering Body");
      Serial.print("BOTTOM LIMIT STATE: ");
      Serial.println(button_states[RAISE_LOWER_BOTTOM_LIMIT]);
      while (!button_states[RAISE_LOWER_BOTTOM_LIMIT]) {
        checkButtons();
      }
      commandHBridge(LEAD_SCREW_MOTOR, 0, LEAD_SCREW_DOWN);
      currentState = LOCATE_ZONE_C;

      // LOCATE_ZONE_C
      break;

    case DROP_PACKAGE:
      openClaw();
      delay(500);
      currentState = RAISE_BODY;
      break;

    case TESTING:
      //commandHBridge(CABLE_PULLEY_MOTOR, 255, PULLEY_OUTBOUND);
      while(1){
        value = readToF();
        Serial.println("Distance mm: "); Serial.println(value);
      }

      
    case LOWER_CLAW:
      Serial.println("LOWER_CLAW");
      commandHBridge(CLAW_MOTOR, 255, CLAW_DOWN);
      Serial.println("Claw going down");
      if (button_states[CLAW_LOWER_LIMIT]) {
        commandHBridge(CLAW_MOTOR, 0, CLAW_DOWN);
        currentState = RAISE_CLAW;
      }
      break;
    
    // ZONE C STATES

    case ZONE_C_TRANSLATE_RIGHT :
      commandHBridge(OMNI_RIGHT, 255, OMNI_FORWARDS);
      commandHBridge(OMNI_LEFT, 0, OMNI_FORWARDS);
      if (button_states[LEFT_OUTER_LIMIT]) {
        commandHBridge(OMNI_RIGHT, 0, OMNI_FORWARDS);
        commandHBridge(OMNI_LEFT, 0, OMNI_FORWARDS);
        currentState = PUT_DOWN;
      }
      if (millis() - zone_C_start_time > DEPOSIT_TIMEOUT) {
        commandHBridge(OMNI_RIGHT, 0, OMNI_BACKWARDS);
        commandHBridge(OMNI_LEFT, 0, OMNI_BACKWARDS);
        currentState = PUT_DOWN;
      }

      break;
    
    case ZONE_C_ROTATE_LEFT:
      Serial.println("ZONE_C_ROTATE_LEFT");
      
      if (button_states[RIGHT_INNER_LIMIT]) {
        currentState = ZONE_C_TRANSLATE_RIGHT;
      } else {
        commandHBridge(OMNI_LEFT, 255, OMNI_FORWARDS);
        commandHBridge(OMNI_RIGHT, 70, OMNI_BACKWARDS);
        Serial.println(button_states[RIGHT_OUTER_LIMIT] && button_states[LEFT_OUTER_LIMIT]);
      }

      if (button_states[RIGHT_OUTER_LIMIT] && button_states[LEFT_OUTER_LIMIT]) {
        commandHBridge(OMNI_LEFT, 0, OMNI_BACKWARDS);
        commandHBridge(OMNI_RIGHT, 0, OMNI_BACKWARDS);
        currentState = PUT_DOWN;
      } else if (button_states[LEFT_OUTER_LIMIT]) {
        currentState = ZONE_C_ROTATE_RIGHT;
      }
      if (millis() - zone_C_start_time > DEPOSIT_TIMEOUT) {
        commandHBridge(OMNI_RIGHT, 0, OMNI_BACKWARDS);
        commandHBridge(OMNI_LEFT, 0, OMNI_BACKWARDS);
        currentState = PUT_DOWN;
      }

      break;

    case ZONE_C_ROTATE_RIGHT:
      Serial.println("ZONE_C_ROTATE_RIGHT");
      commandHBridge(OMNI_LEFT, 70, OMNI_BACKWARDS);
      commandHBridge(OMNI_RIGHT, 255, OMNI_FORWARDS);
      Serial.println(button_states[RIGHT_OUTER_LIMIT] && button_states[LEFT_OUTER_LIMIT]);

      if (button_states[RIGHT_OUTER_LIMIT] && button_states[LEFT_OUTER_LIMIT]) {
        commandHBridge(OMNI_LEFT, 0, OMNI_BACKWARDS);
        commandHBridge(OMNI_RIGHT, 0, OMNI_BACKWARDS);
        currentState = PUT_DOWN;
      } else if (button_states[RIGHT_OUTER_LIMIT]) {
        currentState = ZONE_C_ROTATE_LEFT;
      }

      if (millis() - zone_C_start_time > DEPOSIT_TIMEOUT) {
        commandHBridge(OMNI_RIGHT, 0, OMNI_BACKWARDS);
        commandHBridge(OMNI_LEFT, 0, OMNI_BACKWARDS);
        currentState = PUT_DOWN;
      }

      
      break;

    case LOCATE_ZONE_C:
      Serial.println("LOCATE_ZONE_C");
      if (!started_locating_zone_C) {
        // start timer - if all else fails, have to drop package after certain
        // time
        zone_C_start_time = millis();
        started_locating_zone_C = true;
      }
      // move motors towards zone C. If lined up, move to next stage.
      commandHBridge(OMNI_LEFT, 255, OMNI_FORWARDS);
      commandHBridge(OMNI_RIGHT, 255, OMNI_FORWARDS);
      /*
      if (button_states[LEFT_OUTER_LIMIT]) {
        commandHBridge(OMNI_LEFT, 0, OMNI_FORWARDS);
        Serial.println("Stopping Left Wheel");
      } else {
        commandHBridge(OMNI_LEFT, 255, OMNI_FORWARDS);
        Serial.println("Starting Left Wheel");
      }
      if (button_states[RIGHT_OUTER_LIMIT]) {
        commandHBridge(OMNI_RIGHT, 0, OMNI_FORWARDS);
        Serial.println("Stopping Right Wheel");
      } else {
        commandHBridge(OMNI_RIGHT, 255, OMNI_FORWARDS);
        Serial.println("Starting Right Wheel");
      }
      Serial.println("LOCATE_ZONE_C");
      */
      Serial.println(button_states[RIGHT_OUTER_LIMIT]);
      Serial.println(button_states[LEFT_OUTER_LIMIT]);
      if (button_states[RIGHT_OUTER_LIMIT]) {
        currentState = ZONE_C_ROTATE_LEFT;
      } else if (button_states[LEFT_OUTER_LIMIT]) {
        currentState = ZONE_C_ROTATE_RIGHT;
      }
      
      //currentState = locateZoneC() ? ZONE_C_FINE_TUNING : LOCATE_ZONE_C;
      /*
      Serial.println("Inside EXTEND_ARM case");
      delay(1000);      
      if (!hasPackage) {
        currentState = PICK_UP;
      } else {
        if (hasFoundZoneC) {
          currentState = PUT_DOWN;
        } else {
          currentState = ORIENTATE_BODY_ZONE_C;
        }
      }
      */
      if (millis() - zone_C_start_time > DEPOSIT_TIMEOUT) {
        commandHBridge(OMNI_RIGHT, 0, OMNI_BACKWARDS);
        commandHBridge(OMNI_LEFT, 0, OMNI_BACKWARDS);
        currentState = PUT_DOWN;
      }

      break;

    case PICK_UP :
      Serial.println("Inside PICK_UP case");
      closeClaw();
      hasPackage = true;
      currentState = RAISE_CLAW;

      break;

    case PUT_DOWN : 
      Serial.println("Inside PUT_DOWN case");
      openClaw();
      currentState = RETRACT_ARM; // RETRACT_ARM
      break;

    case ORIENTATE_BODY_ZONE_C :
      hasTranslated = true;
      Serial.println("SWIVELING!");
      
      commandHBridge(SWIVEL_MOTOR, 200, SWIVEL_END);
      delay(1000);
      commandHBridge(SWIVEL_MOTOR, 190, SWIVEL_END);
      //delay(5000);
      //commandHBridge(SWIVEL_MOTOR, 250, SWIVEL_END);
      //Serial.println(button_states[ROTATE_END_LIMIT]);
      while (button_states[ROTATE_END_LIMIT]) { // backwards button
        checkButtons();
      }
      commandHBridge(SWIVEL_MOTOR, 0, SWIVEL_END);
      Serial.println("Stopped Swiveling");
      hasPackage = true;
      currentState = LOWER_BODY;
      /*commandHBridge(SWIVEL_MOTOR, 255, 1);
      delay(1000);
      commandHBridge(SWIVEL_MOTOR, 127, 0);
      delay(1000);
      */
     /*
      if (!button_states[ORIENTATE_DEPOSIT_LIMIT]) {
        commandHBridge(SWIVEL_MOTOR, 255, 1);
      } else {
        commandHBridge(SWIVEL_MOTOR, 0, 1);
        currentState = LOCATE_ZONE_C;
      }
      */
       
      // Determine 
      /*
      if (isRaised) {
        currentState = LOWER_BODY;
      } else {
        currentState = LOCATE_ZONE_C;
      }
      */
      break;

    case ORIENTATE_BODY_HOME :
      Serial.println("ORIENTATE_BODY_HOME");
       
      commandHBridge(SWIVEL_MOTOR, 200, SWIVEL_STATE);
      delay(1000);
      commandHBridge(SWIVEL_MOTOR, 190, SWIVEL_STATE);
      //delay(5000);
      //commandHBridge(SWIVEL_MOTOR, 250, SWIVEL_END);
      //Serial.println(button_states[ROTATE_END_LIMIT]);
      while (button_states[ROTATE_START_LIMIT]) { // backwards button
        checkButtons();
      }
      commandHBridge(SWIVEL_MOTOR, 0, SWIVEL_STATE);
      Serial.println("Stopped Swiveling");
      hasPackage = true;
      currentState = INBOUND_CABLE_TRANSLATE;
    
      // Then change to orientate body
      /*
      Serial.println("Inside CABLE_TRANSLATE case");
      cableTranslate();
      if (hasTranslated) {
        hasTranslated = false; 
      } else {
        hasTranslated = true;
      }

      if (hasPackage) {
        currentState = ORIENTATE_BODY_ZONE_C;
      } else {
        currentState = COMPLETE;
      }
      Serial.println("Finished CABLE_TRANSLATE case");
      */
    
      break;
   

    case COMPLETE :
      Serial.println("COMPLETE");
      break;
    }
}
  



// Function to check control variables. 
/*
void checkHasPackage() {
  Serial.println("inside checkHasPackage function");
  Serial.println("giving you time to put finger over lox to simulate wheel being close...");
  delay(5000);
  lox1.rangingTest(&lox1measure, false);
  if (lox1measure.RangeStatus != 4) {  // phase failures have incorrect data
    if ((lox1measure.RangeMilliMeter) < 50) {
      Serial.println("Wheel is in position");
      hasPackage = true;
      delay(1000);
      
    }else {
      Serial.println("Wheel not detected");
      hasPackage = false;
         
    }
    
  }
}

*/

// High level functions. 

float swivel_angle() {
  return (((float) swivel_position)/341.2 - floor(((float) swivel_position)/341.2)) * 360.0; 
}

void orientate_handler() {
  // needs angle target variable (global)
  unsigned long currentTime = micros();
  unsigned long deltaT = ((float) (orientate_previous_time - currentTime) / 1.0e6); 
  orientate_previous_time = currentTime;

  float kp = 0.1;
  float ki = 0.001;
  float kd = 0;

  float error = (swivel_angle() - swivel_target);
  float dedt = (prev_swivel_error - error) / deltaT;
  float error_integral = prev_error_integral + error * deltaT; 
  
  prev_swivel_error = error;
  prev_error_integral = error_integral;

  float pwr = kp * ((float) error) + ki * error_integral + kd * dedt;
  
  int pwm = min(255, fabs(pwr));

  int direction = pwr < 0; // direction might need to be fixed 

  commandHBridge(SWIVEL_MOTOR, pwm, direction);
  Serial.println(error);
  Serial.println(pwr);
}

void openClaw() {
  clawservo.write(CLAW_OPEN_ANGLE);
  hasPackage = true;
}

void closeClaw() { 
  Serial.println("Closing Claw");
  Serial.println("preparing to close claw");
  clawservo.write(CLAW_CLOSED_ANGLE);
  Serial.println("claw closed");
  hasPackage = false;
}


/*
 * Turns on deposit mech motors according to limit switches. Returns true if
 * Zone C has been found.
 */
void locateZoneC() {
  // uses controlOmniWheels(), readToF()
  Serial.println("inside locateZoneC function");
  commandHBridge(OMNI_LEFT, 255*!button_states[LEFT_OUTER_LIMIT], OMNI_FORWARDS);
  commandHBridge(OMNI_RIGHT, 255*!button_states[RIGHT_OUTER_LIMIT], OMNI_FORWARDS);
  Serial.print("LEFT LIMIT: ");
  Serial.println(button_states[LEFT_OUTER_LIMIT]);
  Serial.print("RIGHT LIMIT: ");
  Serial.println(button_states[RIGHT_OUTER_LIMIT]);
  
  //return button_states[LEFT_OUTER_LIMIT] && button_states[RIGHT_OUTER_LIMIT];

    /*
  delay(100);.
  Serial.println("extending arm");
  delay(100);
  Serial.println("ToF1 measurements:");
  delay(100);
  lox1.rangingTest(&lox1measure, false); // pass in 'true' to get debug data printout!
  delay(100);
  if (lox1measure.RangeStatus != 4) {  // phase failures have incorrect data
    while (((lox1measure.RangeMilliMeter) > (50)) |(lox1measure.RangeStatus == 4)){
      lox1.rangingTest(&lox1measure, false); // pass in 'true' to get debug data printout!
      Serial.print("Distance (mm): "); Serial.println(lox1measure.RangeMilliMeter);
      delay(100);
    }
  }
  Serial.println("CLOSE TO WHEEL!");
  delay(100);
  controlOmniWheels('s',0);
  delay(5000);
  */
}

void retractArm() { 
  Serial.println("inside retractArm function");
  // need to determine for how long we will drive back for!
  Serial.println("preparing to retract arm (but we still need to figure out how long for!)");
  commandHBridge(OMNI_LEFT, 255, OMNI_BACKWARDS);
  commandHBridge(OMNI_RIGHT, 255, OMNI_BACKWARDS);
  // to be changed later
  delay(7000);
  commandHBridge(OMNI_LEFT, 0, OMNI_BACKWARDS);
  commandHBridge(OMNI_RIGHT, 0, OMNI_BACKWARDS);
}

void raiseBody() {
  // TODO 
  Serial.println("setting stepper direction for body raise");
  commandHBridge(LEAD_SCREW_MOTOR, 255, LEAD_SCREW_UP);
  delay(5500);
  commandHBridge(LEAD_SCREW_MOTOR, 0, LEAD_SCREW_UP);
}

void lowerBody() {
  Serial.println("setting stepper direction for body lowering");
  commandHBridge(LEAD_SCREW_MOTOR, 255, LEAD_SCREW_DOWN);
  delay(5500);
  commandHBridge(LEAD_SCREW_MOTOR, 0, LEAD_SCREW_DOWN);
}

void cableTranslate() {
  // uses controlStepper(), readToF()

}

void orientate() {
  // uses controlStepper(), readMag()
  Serial.println("Inside the orientate function");
  while(1){}

}



// Low level functions. 

void commandHBridge(uint8_t motor, uint16_t speed, uint8_t dir) {
    digitalWrite(MOTORS[motor][MOTOR_IN1], dir);
    digitalWrite(MOTORS[motor][MOTOR_IN2], !dir);
    if (speed == 0) {
      digitalWrite(MOTORS[motor][MOTOR_IN1], 0);
      digitalWrite(MOTORS[motor][MOTOR_IN2], 0);
    }
    //Serial.print("Speed:");
    //Serial.println(speed * MOTOR_SCALES[motor] / ((uint16_t) 255));
    analogWrite(MOTORS[motor][MOTOR_ENABLE], speed * MOTOR_SCALES[motor] / ((uint16_t) 255));
}






// Returns the distance the robot is above the ground
uint16_t readToF() {
  return sensor.readRangeSingleMillimeters();
 // VL53L0X_RangingMeasurementData_t measure;
  //lox1.rangingTest(&measure, false);
  //return measure.RangeMilliMeter;
  /*if (ToFnum == 1){
    lox2.stopRangeContinuous();
    lox1.startRangeContinuous();
    int loxmeasurment = loxmeasure1.RangeMilliMeter;
  } else if (ToFnum == 2){
    lox1.stopRangeContinuous();
    lox2.startRangeContinuous();
    int loxmeasurement = loxmeasure2.RangeMilliMeter;
  } else {
    lox1.stopRangeContinuous();
    lox2.stopRangeContinuous();
    int loxmeasurement = 0;
  }
  return loxmeasurement;*/
}

// returns bearing from magnetometer in degrees

/*
 * Debounces button inputs and updates button state array
 */
void checkButtons() {
    for (uint8_t i = 0; i < NUMBER_OF_BUTTONS; i++) {
        if (!digitalRead(buttons_pins[i]) && button_times[i] - millis() > BUTTON_TIME) {
            button_states[i] = true;
        } else if (digitalRead(buttons_pins[i])) {
            button_states[i] = false;
            button_times[i] = millis();
        }
        /*
        Serial.print("Button ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(button_states[i]);
        */
    }
}
