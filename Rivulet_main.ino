/*
  Robot - Rivulet V2.0
  RC Controller - Arduino FS-I6s Demo, FS-IA6B Receiver
  Algorithm - Finite State Machine to control the robot's states
  Microcontroller - Adafruit Feather M4 CAN Express
  Wheels - Rollor coaster type wheels
      Motors - My Actuator Servo - RMD_4015-L_20T (MC CAN) for the wheels
             - My Actuator Servo - RMD_5015-L_10T (MC CAN) for the Robot's Balance
  Linear actuator - Firgelli Utility Linear Actuator
      Motor Driver - PN00218-CYT# Cytron 13A DC Motor Driver
  Mode of Communication - CAN BUS - CAN commands to motors (servo) and PWM signals to other components
*/

// ########################################################################################################################################################################
// DECLERATIONS //
// Include required libraries
#include "CytronMotorDriver.h"
#include <CANSAME5x.h>

//Input Connections
#define CH1 0 // Rx
#define CH2 1 // Tx
// #define CH3 21 //SDA (Digital pin - 21) (Not used)
#define CH5 22 //SCL (Digital pin - 22)
#define CH6 A4 // 

//// Configure the motor drivers for Linear Actuators (1 - 3).
CytronMD la_1(PWM_DIR, 12, 11);  // PWM = Pin 12, DIR = Pin 11 - Linear Actuator 1.
CytronMD la_2(PWM_DIR, 10, 9);  // PWM = Pin 10, DIR = Pin 09 - Linear Actuator 2.
CytronMD la_3(PWM_DIR, 6, 5);  // PWM = Pin 06, DIR = Pin 05 - Linear Actuator 3.

int actuator_speed = 32; // Set the actuators speed here
int wheel_speed = 1000; // Set the wheels Speed in Dps(degrees per second here)

// Integers to represent values from joy sticks
int ch1Value; // Values from Joystick (left and right)
int ch2Value; // Values from Joystick (Up and down)
// int ch3Value;  (Not used)
int ch5Value; // Values from Switch SWB

bool ch6Value; // Boolean to represent the switch value SWA of RC controller
CANSAME5x CAN; // Initalizing CAN object

typedef void (*STATE_HANDLER_T)();    // Defined type to store pointer to a state handler function

void forward();                         // Forward declarations of state handler functions
void reverse();
void linear_actuate_up();
void linear_actuate_down();
void idle();

STATE_HANDLER_T prior_state, state;   // Global variables to store the prior and current states

// ##########################################################################################################################################################################
// SUPPORT FUNCTIONS //

// Read the number of a specified channel and convert to the range provided. // If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue; 
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

// Converts the given speed to haxadecimal and returns the CAN Command for the motor
uint8_t* convert_to_Can(int32_t speed){

  static uint8_t can_command[8];
  int32_t speed_Decimal = (int32_t)(speed/ 0.01);

  can_command[0] = 0xA2; // Command byte
  can_command[1] = 0x00; // NULL
  can_command[2] = 0x00; // NULL
  can_command[3] = 0x00; // NULL

  can_command[4] = (uint8_t)(speed_Decimal & 0xFF);        // Low byte
  can_command[5] = (uint8_t)((speed_Decimal >> 8) & 0xFF); // Byte 5
  can_command[6] = (uint8_t)((speed_Decimal >> 16) & 0xFF);// Byte 6
  can_command[7] = (uint8_t)((speed_Decimal >> 24) & 0xFF);// High byte

  // Return the array of CAN command bytes
  return can_command;  
}

// ############################################################################################################################################################################
// All STATES - (FORWARD, REVERSE , IDLE, LINEAR_ACTUATE_UP, LINEAR_ACTUATE-DOWN) //

// FORWARD STATE - Moves the Robot in forward direction at the given speed(default 1 m/s) --------------------------------------------------------------------------------------------------------------
void forward() {
  if (state != prior_state) {      // If we are entering the state, do initialization stuff
    prior_state = state;
  }

  // Perform state tasks
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch6Value = readSwitch(CH6, false);
  static uint8_t data[8]; // Initializing the data array to recieve/ set CAN commands

  if(ch6Value == HIGH && ch1Value > 10){ 
     if (ch1Value > 85) {
      // Set the predefined values for the static data array
      data[0] = 0xA2;
      data[1] = 0x00;
      data[2] = 0x00;
      data[3] = 0x00;
      data[4] = 0x60;
      data[5] = 0x79;
      data[6] = 0xFE;
      data[7] = 0xFF;  // Command to set motor speed (Modify the data bytes as needed)
    }else {
      int32_t speed = (int32_t)(((float)ch1Value / 100.0) * -wheel_speed); // mapping the speed in dps based on Joy stick value
      memcpy(data, convert_to_Can(speed), 8);  // Copy the result into the `data` array
    }
    
    Serial.println("Robot Moving Forward ... ");
    CAN.beginPacket(0x141);  // Send CAN message with ID 0x141
    for (int i = 0; i < 8; i++) {
      CAN.write(data[i]);
    }
    CAN.endPacket();
  }

  // Check for state transitions
  if (ch1Value < -10) {
    state = reverse;
  }
  else if (abs(ch1Value) < 10){
    state = idle;
  }
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// REVERSE STATE - Moves the Robot in backward/reverse direction at the given speed(default 1 m/s) --------------------------------------------------------------------------------------------------------------
void reverse() {
  if (state != prior_state) {      // If we are entering the state, do initialization stuff
    prior_state = state;
  }

  // Perform state tasks
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch6Value = readSwitch(CH6, false);
  static uint8_t data[8]; // Initializing the data array to recieve/ set CAN commands

  if(ch6Value == HIGH && ch1Value < -10){
    if (ch1Value < -85) {
      // Set the predefined values for the static data array
      data[0] = 0xA2;
      data[1] = 0x00;
      data[2] = 0x00;
      data[3] = 0x00;
      data[4] = 0xA0;
      data[5] = 0x86;
      data[6] = 0x01;
      data[7] = 0x00;  // Command to set motor speed (Modify the data bytes as needed)
    }else {
      int32_t speed = (int32_t)(((float)ch1Value / 100.0) * -wheel_speed); // mapping the speed in dps based on Joy stick value
      memcpy(data, convert_to_Can(speed), 8);  // Copy the result into the `data` array
    }

  Serial.println("Robot Moving Backward ... ");
  CAN.beginPacket(0x141);  // Send CAN message with ID 0x141
  for (int i = 0; i < 8; i++) {
    CAN.write(data[i]);
  }
  CAN.endPacket();
  }

  // Check for state transitions
  if (ch1Value > 10) {
    state = forward;
  }
  else if(abs(ch1Value) < 10){
    state = idle;
  }
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// LINEAR ACTUATE UP - Moves/ opens up the actuators for passing over the obstacles -------------------------------------------------------------------------------------------------------------
void linear_actuate_up() {
  if (state != prior_state) {      // If we are entering the state, do initialization stuff
    prior_state = state;
  }

  // Perform state tasks
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch5Value = readChannel(CH5, -100, 100, 0);
  ch6Value = readSwitch(CH6, false);

  if(ch6Value == LOW && ch2Value > 10){   
    if (ch5Value < -10){
      Serial.println("Opening up Actuator 1 ... ");
      la_1.setSpeed(actuator_speed);
    } 
    else if (ch5Value > 10){
      Serial.println("Opening up Actuator 3 ... ");
      la_3.setSpeed(actuator_speed);
    }
    else{
      Serial.println("Opening up Actuator 2 ... ");
      la_2.setSpeed(actuator_speed);
    }
  }

  // Check for state transitions
  if (ch6Value == HIGH || (abs(ch2Value) < 10)) {
    state = idle;
  }
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// LINEAR ACTUATE DOWN - Moves/ closes down the actuators for locking the wheels on the rod/pivot -------------------------------------------------------------------------------------------------------------
void linear_actuate_down() {
  if (state != prior_state) {      // If we are entering the state, do initialization stuff
    prior_state = state;
  }

  // Perform state tasks
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch5Value = readChannel(CH5, -100, 100, 0);
  ch6Value = readSwitch(CH6, false);

  if(ch6Value == LOW && ch2Value < -10){   
    if (ch5Value < -10){
      Serial.println("Closing down Actuator 1 ... ");
      la_1.setSpeed(-actuator_speed);
    } 
    else if (ch5Value > 10){
      Serial.println("Closing down Actuator 3 ... ");
      la_3.setSpeed(-actuator_speed);
    }
    else{
      Serial.println("Closing down Actuator 2 ... ");
      la_2.setSpeed(-actuator_speed);
    }
  }

  // Check for state transitions
  if (ch6Value == HIGH || (abs(ch2Value) < 10)) {
    state = idle;
  }
}

// IDLE STATE - Maintains the robot in Idle state while reading signals/inputs from the sensors/user ---------------------------------------------------------------------------------------------------------------------------------------------------------
void idle() {
  if (state != prior_state) {      // If we are entering the state, do initialization stuff
    prior_state = state;
  }

  // Perform state tasks
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch5Value = readChannel(CH5, -100, 100, 0);
  ch6Value = readSwitch(CH6, false);

  byte data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};// Command to stop the motor (all speed bytes set to 0)

  Serial.println("Robot in Idle/Stopped State ... ");
  CAN.beginPacket(0x141);  // Send CAN message with ID 0x141
  for (int i = 0; i < 8; i++) {
    CAN.write(data[i]);
  }
  CAN.endPacket();

  // Command to stop the linear Actuators (all speeds set to 0)
  la_1.setSpeed(0);
  la_2.setSpeed(0);
  la_3.setSpeed(0);

  if (ch6Value == HIGH && ch1Value > 10) {
    state = forward;
  }
  else if(ch6Value == HIGH && ch1Value < -10) {
    state = reverse;
  }
  else if(ch6Value == LOW && ch2Value > 10){
    state = linear_actuate_up;
  }
  else if(ch6Value == LOW && ch2Value < -10){
    state = linear_actuate_down;
  }
}
// ########################################################################################################################################################################

void setup(){
  // Set up serial monitor
  Serial.begin(115200);
  
  // Set all pins as inputs
  pinMode(CH1, INPUT); // Rx
  pinMode(CH2, INPUT); // Tx
  // pinMode(CH3, INPUT); // SDA  (Not used)
  pinMode(CH5, INPUT); // SCL
  pinMode(CH6, INPUT); // A4

  // Set up CAN standby and booster pins
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false);  // Turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true);   // Turn on booster

  // Initialize the CAN bus at 1 Mbps
  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1) delay(10);
  }
  Serial.println("CAN Initialized!");

  prior_state = NULL;
  state = idle;
}

void loop() {
  state();
}
