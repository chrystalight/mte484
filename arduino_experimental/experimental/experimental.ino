#include <Arduino.h>
#include "geeWhiz.h"

// =============== Control System Parameters
int T = 10; //sampling time in MS
float U = 5.0; //reference signal, will be overwritten
int max_T = 5000;

// NEW: State machine to control the program flow
enum ProgramState { WAIT_FOR_INPUT, RUNNING_TEST, TEST_COMPLETE };
volatile ProgramState currentState = WAIT_FOR_INPUT;

volatile int i = 0;
// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

//=================== Potentiometer Calibration ====
int pot_min = 565;
int pot_max = 455;
int pot_mid = 508;

double m = M_PI/(2*(pot_max-pot_min));
double offset = M_PI/4-m*pot_max;

double map_potentiometer(int val){
  double radians = m*val+offset;
  return radians;
}

// ================== Setup ==================
void setup() {
  pinMode(A5, OUTPUT);
  Serial.begin(115200);
  delay(1000);

  geeWhizBegin();                 
  set_control_interval_ms(T);
  setMotorVoltage(0);

  Serial.println("Setup complete.");
  Serial.println("Please enter a value for the step input (U) and press Enter:");
}

// ================== Loop ==================
void loop() {
  // The main loop now acts as a state manager
  
  if (currentState == WAIT_FOR_INPUT) {
    if (Serial.available() > 0) {
      // Read the user's desired R value
      U = Serial.parseFloat();

      // Clear the serial buffer of any extra characters
      while(Serial.available() > 0) { Serial.read(); }

      // Prepare for the new test
      Serial.println("==============================================================================");
      Serial.print("NEW TEST STARTED -> STEP INPUT VAL (U): ");
      Serial.println(U);
      Serial.print("SAMPLING TIME VAL (T): ");
      Serial.println(T);
      Serial.println("==============================================================================");
      Serial.println("Time (ms), Reference, Angle (rad), Raw Sensor");
      
      i = 0; // Reset the timer/counter
      currentState = RUNNING_TEST; // Change state to start the test
    }
  } 
  else if (currentState == TEST_COMPLETE) {
    Serial.println("==============================================================================");
    Serial.println("Test complete.");
    Serial.println("\nPlease enter a new value for R to run another test:");
    currentState = WAIT_FOR_INPUT; // Go back to waiting for user input
  }
  
  // If currentState is RUNNING_TEST, do nothing in the main loop.
  // The ISR is handling the experiment.
}

// ================== Control ISR ==================
void interval_control_code(void) {
  // This function only executes the test logic when in the RUNNING_TEST state
  if (currentState != RUNNING_TEST) {
    return; // Do nothing if we are not running a test
  }
  
  // Check if the test duration has been exceeded
  if (T * i >= max_T) {
    setMotorVoltage(0); // Turn off the motor
    currentState = TEST_COMPLETE; // Signal the main loop that the test is finished
    return; // Stop further execution in the ISR
  }

  // ---- Read sensors and apply step input ----
  int motor = analogRead(MOT_PIN);

  // ---- Print data in CSV format ----
  setMotorVoltage(U); // Apply the constant step input voltage
  // ---- Print data in CSV format ----
  Serial.print(T*i);
  Serial.print(",");
  Serial.print(U);
  Serial.print(","); // Print angle in radians with 5 decimal places
  Serial.print(map_potentiometer(motor), 5);
  Serial.print(", ");
  Serial.println(motor);
  i+=1;
}