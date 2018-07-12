/*
 *  DCS Bios version 0.7.1
 * 
 *  Up Front Controller, Hud Annunciators, Altimeter,     
 *  Verticle Speed Indicator, and HSI Bearing Pointer
 *  
 *  Stepper Motor VID29-05P (Can be directly driven by MCU):
 *  -Low current consumption: <20mA
 *  -Rotation Angle:Max 315°
 *  -0.5°/full step
 *  -Coil resistance: 280 +/- 20Ω
 *  Wiring View
 *  Top Right Pin Coil 1 connects to aarduino 2
 *  Bottom Right Pin Coil 2 connects to aarduino 3
 *  Bottom Left Pin Coil 3 connects to aarduino 4
 *  Top Left Pin Coil 4 connects to arduino 5
 */
 
#define DCSBIOS_IRQ_SERIAL
#include <AccelStepper.h>
#include "DcsBios.h"
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

// 0x3F was the 16x2 LCD
// 0x27 is the 20x4 LCD
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 came from running I2C address scanner

// Motor pin definitions
#define mtrPin5  6   // IN1 on the L298N driver #2
#define mtrPin6  7   // IN2 on the L298N driver #2
#define mtrPin7  8   // IN3 on the L298N driver #2
#define mtrPin8  9   // IN4 on the L298N driver #2
#define mtrPin9  10  // IN1 on the L298N Motor driver #3
#define mtrPin10 11  // IN2 on the L298N Motor driver #3
#define mtrPin11 12  // IN3 on the L298N Motor driver #3
#define mtrPin12 13  // IN4 on the L298N Motor driver #3


// number of steps to reach 50,000 Feet
#define ALT_STEPS_PER_REVOLUTION 1000 // 1000 steps is 50,000 ft.  20 steps per 1000 feet
#define ALT_ZERO_OFFSET -9

// Number of steps from stop to stop on VID29-02 Stepper Motor
#define VSI_STEPS_PER_REVOLUTION 630
// number of steps for 1 revolution of real HSI Bearing Pointer
#define BRG_STEPS_PER_REVOLUTION 600 

// Offset is obtained by trial and error.
// Has little to do with actual positioning of the home switch.
#define BRG_ZERO_OFFSET -17

// 4 = Full4wire setup.  PWM pin output is defined in code above
AccelStepper stepper_BRG(4, mtrPin9, mtrPin11, mtrPin10, mtrPin12);
AccelStepper stepper_VSI(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper stepper_ALT(4, mtrPin5, mtrPin7, mtrPin6, mtrPin8);

int BRG_currentStepperPosition = 0;
int ALT_currentStepperPosition = 0; // current stepper position

signed long BRG_lastAccelStepperPosition;
signed long ALT_lastAccelStepperPosition;

// Used to Home Stepper at startup
long BRG_initial_homing = -1;
// Need two because its CW or CCW depending on if POT reading is above or below start point
long ALT_initial_homing = 1;
long ALT_initial_homing2 = -1;

int DH_Flag = A13; // decision height annunciator
int ALT_Flag = A12;  // Altimeter On/Off Flag
int ALT_POT = A11; // stepper motor position potentiometer
int ALT_POT_Val = 0; 
int ALT_POT_HOME = 292; // Home number value changed between sessions. (POT gear was slipping, tighten it to fix)

// Define pin used for proximity switch
#define BRG_home_switch A10

/* UP FRONT CONTROLLER */

// STEER Up/Down
DcsBios::Switch3Pos ufcSteer("UFC_STEER", 23, 22);

// 1
DcsBios::Switch2Pos ufc1("UFC_1", 24);

// 4
DcsBios::Switch2Pos ufc4("UFC_4", 25);

// 7
DcsBios::Switch2Pos ufc7("UFC_7", 26);

// 2
DcsBios::Switch2Pos ufc2("UFC_2", 27);

// 5
DcsBios::Switch2Pos ufc5("UFC_5", 28);

// 8
DcsBios::Switch2Pos ufc8("UFC_8", 29);

// 3
DcsBios::Switch2Pos ufc3("UFC_3", 30);

// 6
DcsBios::Switch2Pos ufc6("UFC_6", 31);

// 9
DcsBios::Switch2Pos ufc9("UFC_9", 32);

// Hack
DcsBios::Switch2Pos ufcHack("UFC_HACK", 33);

// 0
DcsBios::Switch2Pos ufc10("UFC_10", 34);

// Space
DcsBios::Switch2Pos ufcSpc("UFC_SPC", 35);

// FUNC
DcsBios::Switch2Pos ufcFunc("UFC_FUNC", 36);

// LTR
DcsBios::Switch2Pos ufcLtr("UFC_LTR", 37);

// CLR
DcsBios::Switch2Pos ufcClr("UFC_CLR", 38);

// ENT
DcsBios::Switch2Pos ufcEnt("UFC_ENT", 39);

// MK
DcsBios::Switch2Pos ufcMk("UFC_MK", 40);

// ALT ALRT
DcsBios::Switch2Pos ufcAltAlrt("UFC_ALT_ALRT", 41);

// Master Caution Reset Switch
DcsBios::Switch2Pos ufcMasterCaution("UFC_MASTER_CAUTION", 42);

// Master Caution Light
DcsBios::LED mcLed(0x1012, 0x0800, 43);

// FWD
DcsBios::Switch2Pos ufcNa1("UFC_NA1", 44);

// MID
DcsBios::Switch2Pos ufcNa2("UFC_NA2", 46);

// AFT
DcsBios::Switch2Pos ufcNa3("UFC_NA3", 48);

// UNK 4
DcsBios::Switch2Pos ufcNa4("UFC_NA4", 45);

// UNK 5
DcsBios::Switch2Pos ufcNa5("UFC_NA5", 47);

// UNK 6
DcsBios::Switch2Pos ufcNa6("UFC_NA6", 49);

// Data Up/Down
DcsBios::Switch3Pos ufcData("UFC_DATA", 51, 50);

// SEL Up/Down
DcsBios::Switch3Pos ufcSel("UFC_SEL", 53, 52);

// INTEN Incr/Decr
DcsBios::Switch3Pos ufcInten("UFC_INTEN", A0, A1);

// DEPR Up/Down
DcsBios::Switch3Pos ufcDepr("UFC_DEPR", A3, A2);

/*****************************************/

/*  HUD ANNUNCIATORS   */

// AOA Indexer Low
DcsBios::LED aoaIndexerLow(0x1012, 0x4000, A4);

// AOA Indexer Normal
DcsBios::LED aoaIndexerNormal(0x1012, 0x2000, A5);

// AOA Indexer High
DcsBios::LED aoaIndexerHigh(0x1012, 0x1000, A6);

// Air Refuel READY
DcsBios::LED airRefuelReady(0x1012, 0x8000, A7);

// Air Refuel LATCHED
DcsBios::LED airRefuelLatched(0x1026, 0x0100, A8);

// Air Refuel DISCONNECT
DcsBios::LED airRefuelDisconnect(0x1026, 0x0200, A9);

/***************************************/

/* Altimeter */
// Set Baro Pressure
DcsBios::RotaryEncoder altSetPressure("ALT_SET_PRESSURE", "-25600", "+25600", A14, A15);

// Altimeter PNEU Flag
void onAltPneuFlagChange(unsigned int ALT_Flag_newValue) {
  unsigned int ALT_Flag_Value = (ALT_Flag_newValue & 0xffff) >> 0;
  if (ALT_Flag_Value < 25000)
  {
      digitalWrite(ALT_Flag, HIGH);   
  }
  else
  {
      digitalWrite(ALT_Flag, LOW);       
  }
}
DcsBios::IntegerBuffer altPneuFlagBuffer(0x12c8, 0xffff, 0, onAltPneuFlagChange);

void setup() {
  DcsBios::setup();
  pinMode (ALT_Flag, OUTPUT);
  digitalWrite (ALT_Flag, HIGH);

  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RAW ALT:");
  lcd.setCursor(0, 1);
  lcd.print("M:");
  lcd.setCursor(8, 1);
  lcd.print("POT:");     

  /* VSI Stepper Motor Homing*/
  stepper_VSI.setMaxSpeed(1500.0);
  stepper_VSI.setAcceleration(400.0); // Speed needle moves at for homing to stops
  stepper_VSI.runToNewPosition(VSI_STEPS_PER_REVOLUTION);  // Forward sweep to find forward stop, 630 steps
  stepper_VSI.runToNewPosition(0);    // Backward sweep to rear stop
  stepper_VSI.setCurrentPosition(0);  // Establishes the zero position
  stepper_VSI.setSpeed(1000);    // Set a constant speed. use this only if you use runSpeedToPosition() below
  stepper_VSI.setAcceleration(750.0); // Speed needle moves at for simulator game
/*****************************/

/* Bearing Pointer Homing*/
// Homing Code was based on Brainy Bits
// https://www.brainy-bits.com/setting-stepper-motors-home-position-using-accelstepper/

// Bearing Pointer HOMING
  pinMode(BRG_home_switch, INPUT_PULLUP);  // LOW when in zero position, HIGH otherwise

  stepper_BRG.setMaxSpeed(400);
  stepper_BRG.setSpeed(200);
  stepper_BRG.setAcceleration(65);

  while (digitalRead(BRG_home_switch)) {
    stepper_BRG.moveTo(BRG_initial_homing);
    BRG_initial_homing--;
    stepper_BRG.run();
    delay(5);
  }
  stepper_BRG.setCurrentPosition(0);
  stepper_BRG.setMaxSpeed(400);
  stepper_BRG.setSpeed(200);
  stepper_BRG.setAcceleration(65);
  BRG_initial_homing = 1;

  while (!digitalRead(BRG_home_switch)) {
    stepper_BRG.moveTo(BRG_initial_homing);
    stepper_BRG.run();
    BRG_initial_homing++;
    delay(5);
  }
  // Zero offset code came from DCS Forums
  // Add zero offset
  stepper_BRG.runToNewPosition(stepper_BRG.currentPosition() + BRG_ZERO_OFFSET);

  stepper_BRG.setCurrentPosition(0);
  BRG_lastAccelStepperPosition = 0;
  stepper_BRG.setMaxSpeed(600);
  stepper_BRG.setSpeed(400);
  stepper_BRG.setAcceleration(100);
/************************************/
/* Altimeter Homing*/
  // Homing Code was based on Brainy Bits
  // https://www.brainy-bits.com/setting-stepper-motors-home-position-using-accelstepper/
    
  // ALTIMETER HOMING
  // Set Max Speed and Acceleration for startup during homing
  stepper_ALT.setMaxSpeed(15);
  stepper_ALT.setSpeed(5);
  stepper_ALT.setAcceleration(5);

  /* Determine which way to move  */
  
    ALT_POT_Val = analogRead(ALT_POT);
if (ALT_POT_Val >= ALT_POT_HOME)
{
  while (ALT_POT_Val > ALT_POT_HOME)
  {
    stepper_ALT.moveTo(ALT_initial_homing);  // Set the position to move to
    ALT_initial_homing++;  // Decrease by 1 for next move if needed
    stepper_ALT.run();  // Start moving the stepper
    delay(5); 
    ALT_POT_Val = analogRead(ALT_POT);
    lcd.setCursor(12, 1);
    lcd.print("+    ");  
    lcd.setCursor(13, 1);
    lcd.print(ALT_POT_Val);    
  }
  stepper_ALT.setCurrentPosition(0);  // Set the current position as zero for now
  stepper_ALT.setMaxSpeed(25);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper_ALT.setSpeed(10);
  stepper_ALT.setAcceleration(10);  // Set Acceleration of Stepper
  ALT_initial_homing = -1;
  }
else
{
  while (ALT_POT_Val < ALT_POT_HOME)
  {
    stepper_ALT.moveTo(ALT_initial_homing2);  // Set the position to move to
    ALT_initial_homing2--;  // Decrease by 1 for next move if needed
    stepper_ALT.run();  // Start moving the stepper
    delay(5);
    ALT_POT_Val = analogRead(ALT_POT);
    lcd.setCursor(12, 1);
    lcd.print("-    ");  
    lcd.setCursor(13, 1);
    lcd.print(ALT_POT_Val);     
  }
  stepper_ALT.setCurrentPosition(0);  // Set the current position as zero for now
  stepper_ALT.setMaxSpeed(25);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper_ALT.setSpeed(10);
  stepper_ALT.setAcceleration(10);  // Set Acceleration of Stepper
  ALT_initial_homing2 = 1;
 } 
  // Zero offset code came from DCS Forums
  // Add zero offset
  stepper_ALT.runToNewPosition(stepper_ALT.currentPosition() + ALT_ZERO_OFFSET);

  // tell the AccelStepper library that we are at the ZERO ALT
  stepper_ALT.setCurrentPosition(0);  // Set the current position as zero for now
  ALT_lastAccelStepperPosition = 0;
}

void loop() {
  DcsBios::loop();

  // move stepper motors
  stepper_VSI.runSpeedToPosition(); // move stepper motor (constant speed)
  stepper_BRG.run();
  stepper_ALT.run();    
}

/* VSI*/
void onVviChange(unsigned int new_VSI_Value) {
  unsigned int vsiValue = (new_VSI_Value & 0xffff) >> 0;
  unsigned int Gauge_VSI = map(vsiValue, 0, 65535, VSI_STEPS_PER_REVOLUTION, 0);     
  stepper_VSI.moveTo(Gauge_VSI);   //set an absolute target
  
}
DcsBios::IntegerBuffer vviBuffer(0x106e, 0xffff, 0, onVviChange);

// HSI Bearing Pointer 1
void onHsiBearing1Change(unsigned int new_BRG_Value) {
  unsigned int Bearing_Value = (new_BRG_Value & 0xffff) >> 0;
  int BRG_targetPosition = map(Bearing_Value, 0, 65535, BRG_STEPS_PER_REVOLUTION - 1, 0);

  // Copied this section of code from DCS forums
  
  int BRG_movementSinceLastUpdate = stepper_BRG.currentPosition() - BRG_lastAccelStepperPosition;
  BRG_currentStepperPosition += BRG_movementSinceLastUpdate;
  BRG_lastAccelStepperPosition = stepper_BRG.currentPosition();

  if (BRG_currentStepperPosition < 0) BRG_currentStepperPosition += BRG_STEPS_PER_REVOLUTION;
  if (BRG_currentStepperPosition > BRG_STEPS_PER_REVOLUTION) BRG_currentStepperPosition -= BRG_STEPS_PER_REVOLUTION;

  int BRG_delta = BRG_targetPosition - BRG_currentStepperPosition;

  // if we would move more than 180 degree counterclockwise, move clockwise instead
  if (BRG_delta < -(BRG_STEPS_PER_REVOLUTION / 2)) BRG_delta += BRG_STEPS_PER_REVOLUTION;

  // if we would move more than 180 degree clockwise, move counterclockwise instead
  if (BRG_delta > (BRG_STEPS_PER_REVOLUTION / 2)) BRG_delta -= BRG_STEPS_PER_REVOLUTION;

  // tell AccelStepper to move relative to the current position
  stepper_BRG.move(BRG_delta);
}
DcsBios::IntegerBuffer hsiBearing1Buffer(0x104e, 0xffff, 0, onHsiBearing1Change);

// Altitude
void onAltMslFtChange(unsigned int new_ALT_Value) {
  unsigned int ALT_Value = (new_ALT_Value & 0xffff) >> 0;

// Started with 2622 but kept lowering it by trial and error till it matched sim.  2535 worked best

  int ALT_targetPosition = -(map(ALT_Value, 0, 65535, 0, 2535));

  lcd.setCursor(9, 0);
  lcd.print("       ");  
  lcd.setCursor(9, 0);
  lcd.print(ALT_Value);

  lcd.setCursor(3, 1);
  lcd.print("    ");  
  lcd.setCursor(3, 1);
  lcd.print(ALT_targetPosition);

/*
 *  alt msl values are 0 ft to 65535 ft
 *  every 20 steps is 1000 ft 
 *  65536/1000  = 65.536
 *  65*20 = 1311
 *  Above calculations was off. The closer number was 2622 for mapping
 */

  // adjust ALT_currentStepperPosition to include the distance our stepper motor
  // was moved since we last updated it
  // Copied this section of code from DCS forums
  
  int ALT_movementSinceLastUpdate = stepper_ALT.currentPosition() - ALT_lastAccelStepperPosition;
  ALT_currentStepperPosition += ALT_movementSinceLastUpdate;
  ALT_lastAccelStepperPosition = stepper_ALT.currentPosition();

  if (ALT_currentStepperPosition < 0) ALT_currentStepperPosition += ALT_STEPS_PER_REVOLUTION;
  if (ALT_currentStepperPosition > ALT_STEPS_PER_REVOLUTION) ALT_currentStepperPosition -= ALT_STEPS_PER_REVOLUTION;

  int ALT_delta = ALT_targetPosition - ALT_currentStepperPosition;

  // if we would move more than 180 degree counterclockwise, move clockwise instead
  if (ALT_delta < -(ALT_STEPS_PER_REVOLUTION / 2)) ALT_delta += ALT_STEPS_PER_REVOLUTION;

  // if we would move more than 180 degree clockwise, move counterclockwise instead
  if (ALT_delta > (ALT_STEPS_PER_REVOLUTION / 2)) ALT_delta -= ALT_STEPS_PER_REVOLUTION;

  // tell AccelStepper to move relative to the current position
  stepper_ALT.move(ALT_delta);
}
DcsBios::IntegerBuffer altMslFtBuffer(0x0408, 0xffff, 0, onAltMslFtChange);

