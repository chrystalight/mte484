#include <Arduino.h>
#include "geeWhiz.h"

//INSTRUCTIONS:
//TO ADJUST KP, UNCOMMENT THE RIGHT LINE AND RECOMPILE AND RE UPLOAD.
//SAME FOR T

//MAKE SURE INPUT MODE AND SATURATOR ACTIVE ARE SET CORRECTLY -- INPUT MODE 1 WILL BE WHAT YOU WANT

// =============== System and Control Parameters ===============
int T = 3;                                  // Sampling time in MS
float Kp = -19.2928;                        // Proportional gain for each calculated zeta
// float Kp = -3.3494;
// float Kp = -4.8232;
// float Kp = -9.6483;
float stiction_offset_neg = -0.202;
float stiction_offset_pos = 0.236;
int max_T = 10000;                          // Test duration in MS

// --- Input Parameters ---
float frequency_hz;                   // Default for sine wave mode
const float amplitude_rad = M_PI / 8.0;
float step_magnitude_rad;             // Default for step input mode
float open_loop_voltage;              // Default for open-loop mode

// --- Saturator Limits ---
const float saturation_limit = 0.7;         // The limit for the reference signal in radians

// =============== Mode setup ===============
// Set the desired mode for the controller.
// 0 --> Open-loop control. 
// 1 --> Step input mode.
// 2 --> Sine wave tracking mode. 
int input_mode = 1;
bool saturator_active = false;

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// State machine to control the program flow
enum ProgramState { WAIT_FOR_INPUT, RUNNING_TEST, TEST_COMPLETE };
volatile ProgramState currentState = WAIT_FOR_INPUT;

volatile int i = 0;

//=================== Potentiometer Calibration ====
int pot_min = 565;
int pot_max = 455;
double m = M_PI / (2.0 * (pot_max - pot_min));
double offset = M_PI / 4.0 - m * pot_max;

double map_potentiometer(int val) {
  double radians = m * val + offset;
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

  Serial.println("==================================================");
  Serial.print("Proportional Gain (Kp) = ");
  Serial.println(Kp);
  Serial.print("Saturator Active: ");
  Serial.println(saturator_active ? "YES" : "NO");

  // Check input mode and display the appropriate prompt
  if (input_mode == 0) {
    Serial.println("Mode: OPEN-LOOP CONTROL");
    Serial.println("\nPlease enter a MOTOR VOLTAGE (Vin) and press Enter:");
  } else if (input_mode == 1) {
    Serial.println("Mode: STEP INPUT");
    Serial.println("\nPlease enter a STEP MAGNITUDE (in radians) and press Enter:");
  } else if (input_mode == 2) {
    Serial.println("Mode: SINE WAVE TRACKING");
    Serial.println("\nPlease enter a SINE WAVE FREQUENCY (in Hz) and press Enter:");
  }
  Serial.println("==================================================");
}

// ================== Loop ==================
void loop() {
  if (currentState == WAIT_FOR_INPUT) {
    if (Serial.available() > 0) {
      float userInput = Serial.parseFloat();
      while (Serial.available() > 0) { Serial.read(); }

      Serial.println("==============================================================================");
      Serial.print("Proportional Gain (Kp) = ");
      Serial.println(Kp);
      if (input_mode == 0) {
        open_loop_voltage = userInput;
        Serial.print("NEW TEST STARTED -> OPEN-LOOP VOLTAGE: ");
        Serial.print(open_loop_voltage, 4);
        Serial.println(" V");
      } else if (input_mode == 1) {
        step_magnitude_rad = userInput;
        Serial.print("NEW TEST STARTED -> STEP MAGNITUDE: ");
        Serial.print(step_magnitude_rad, 4);
        Serial.println(" rad");
      } else if (input_mode == 2) {
        frequency_hz = userInput;
        Serial.print("NEW TEST STARTED -> SINE WAVE FREQUENCY: ");
        Serial.print(frequency_hz, 2);
        Serial.println(" Hz");
      }

      Serial.println("==============================================================================");
      Serial.println("Time (ms),Original Ref (V or rad),Final Ref (V or rad),Angle (rad),Raw Sensor");
      
      i = 0;
      currentState = RUNNING_TEST;
    }

  } else if (currentState == TEST_COMPLETE) {
    Serial.println("==============================================================================");
    Serial.println("Test complete.");
    
    if (input_mode == 0) {
        Serial.println("\nPlease enter a new voltage to run another test:");
    } else if (input_mode == 1) {
        Serial.println("\nPlease enter a new step magnitude to run another test:");
    } else if (input_mode == 2) {
        Serial.println("\nPlease enter a new frequency to run another test:");
    }
    currentState = WAIT_FOR_INPUT;
  }
}

// ================== Control ISR ==================
void interval_control_code(void) {
  if (currentState != RUNNING_TEST) {
    return;
  }
  
  if (T * i >= max_T) {
    setMotorVoltage(0);
    currentState = TEST_COMPLETE;
    return;
  }

  // ---- Read sensor ----
  int motor_raw = analogRead(MOT_PIN);
  double current_angle = map_potentiometer(motor_raw);

  double original_ref = 0;
  double final_ref = 0;
  double U = 0;

  // ---- Control Logic based on Mode ----
  if (input_mode == 0) {
    // Mode 0: Direct open-loop voltage control.
    U = open_loop_voltage;
    original_ref = U; // Log the commanded voltage for reference
    final_ref = U;    // Log the commanded voltage for reference
  } else {

    // Modes 1 & 2: Closed-loop control.

    // --- Generate the reference signal based on the selected mode ---
    if (input_mode == 1) {
      original_ref = step_magnitude_rad;
    } else { // input_mode == 2, calculate sin wave
      float current_time_s = (float)(T * i) / 1000.0;
      original_ref = amplitude_rad * sin(2.0 * M_PI * frequency_hz * current_time_s);
    }

    // ---- Apply Saturator to the reference signal if active ----
    final_ref = original_ref;
    if (saturator_active) {
      if (final_ref > saturation_limit) {
        final_ref = saturation_limit;
      } else if (final_ref < -saturation_limit) {
        final_ref = -saturation_limit;
      }
    }

    // ---- Proportional Control Law ----
    double error = final_ref - current_angle;
    U = Kp * error;

    // ---- Add Stiction Compensation ----
    if (abs(U) > 0.01) {
      U = U + (U > 0 ? stiction_offset_pos : stiction_offset_neg);
    }
  }
  
  // Apply the calculated voltage U to the motor
  setMotorVoltage(U);
  
  // ---- Print data in CSV format ----
  Serial.print(T * i);
  Serial.print(",");
  Serial.print(original_ref, 5); 
  Serial.print(",");
  Serial.print(final_ref, 5);
  Serial.print(",");
  Serial.print(current_angle, 5);
  Serial.print(",");
  Serial.println(motor_raw);
  
  i++;
}