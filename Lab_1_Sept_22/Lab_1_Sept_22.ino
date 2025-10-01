#include <Arduino.h>
#include "geeWhiz.h"

// =============== Control System Parameters
float T = 1.4; //sampling time in MS
float U = 5.0; //reference signal, WILL BE OVERWRITTEN BY USER INPUT
int max_T = 5000;
volatile int i = 0;
// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

//=================== Potentiometer Calibration ====
//setup a default value for the potentiometer calculations that will be overwritten if calibrate_potentiometer is called later
int pot_min = 565;
int pot_max = 455;
int pot_mid = 508;

double m = M_PI/(2*(pot_max-pot_min));
double offset = M_PI/4-m*pot_max;

double map_potentiometer(int val){
  double radians = m*val+offset;
  return radians;
}

void calibrate_potentiometer(){
  Serial.println("Please move the motor to the most negative position (-45 degrees, down).");
  Serial.println("Taking potentiometer reading in 10 seconds...");
  delay(10000);
  
  // Read the minimum value over 4 seconds and average it
  Serial.println("Reading minimum value...");
  long sum_min = 0;
  int num_readings = 400; // Take 400 readings over 4 seconds
  for (int i = 0; i < num_readings; i++){
    sum_min += analogRead(MOT_PIN);
    delay(10); // 400 readings * 10ms delay = 4000ms = 4 seconds
  }
  pot_min = sum_min / num_readings;
  Serial.print("Minimum potentiometer value set to: ");
  Serial.println(pot_min);
  Serial.println("------------------------------------");


  Serial.println("Please move the motor to the most positive position (45 degrees, up).");
  Serial.println("Taking potentiometer reading in 10 seconds...");
  delay(10000);
  
  // Read the maximum value over 4 seconds and average it
  Serial.println("Reading maximum value...");
  long sum_max = 0;
  for (int i = 0; i < num_readings; i++){
    sum_max += analogRead(MOT_PIN);
    delay(10);
  }
  pot_max = sum_max / num_readings;
  Serial.print("Maximum potentiometer value set to: ");
  Serial.println(pot_max);
  Serial.println("------------------------------------");

  // Recalculate m and offset using the new calibrated values
  m = M_PI / (2.0 * (pot_max - pot_min));
  offset = (M_PI / 4.0) - m * pot_max;

  Serial.println("Calibration complete!");
  Serial.print("New m: ");
  Serial.println(m, 6); // Print with 6 decimal places for precision
  Serial.print("New offset: ");
  Serial.println(offset, 6);
}
  
// ================== MODIFIED Setup ==================
void setup() {

  pinMode(A5, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  //calibrate_potentiometer(); //comment out this line to use the defaults

  // Wait for user to input a value for R
  Serial.println("Please enter a value for the step input (R) and press Enter:");
  while (Serial.available() == 0) {
    delay(50); // Wait patiently for input
  }

  // Read the floating point number and set it as our reference signal R
  R = Serial.parseFloat();
  Serial.println("==============================================================================");
  Serial.print("STEP INPUT VAL (R): ");
  Serial.println(R);
  Serial.print("SAMPLING TIME VAL (T): ");
  Serial.println(T);
  Serial.println("==============================================================================");
  Serial.println("T, U, Y, H");

  // Now, continue with the rest of the setup
  geeWhizBegin();                 
  set_control_interval_ms(T); // Set the sampling time from the global variable
  setMotorVoltage(0);

  Serial.println("geeWhiz Started");

}

// ================== Loop ==================
void loop() {
  // The main loop is empty as all work is done in the timer interrupt (ISR)
}


// ================== Control ISR ==================
void interval_control_code(void) {
  // This function runs automatically every 'T' milliseconds
  // Stop the test after the max time has elapsed
  if (T * i > max_T) {
    setMotorVoltage(0); // Turn off the motor
    return; // Do nothing further
  }

  //Serial.print(micros());
  // ---- Read sensors and apply step input ----
  int motor = analogRead(MOT_PIN);
  setMotorVoltage(U); // Apply the constant step input voltage
  // ---- Print data in CSV format ----
  Serial.print(T);
  Serial.print(",");
  Serial.print(U);
  Serial.print(","); // Print angle in radians with 5 decimal places
  Serial.print(map_potentiometer(motor), 5);
  Serial.print(", ");
  Serial.print(motor);
  i+=1;
}