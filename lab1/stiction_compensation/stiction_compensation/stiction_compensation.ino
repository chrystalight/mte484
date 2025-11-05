#include <Arduino.h>
#include "geeWhiz.h"

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

//=================== Potentiometer Calibration ====
int pot_min = 565;
int pot_max = 455;
int pot_mid = 508;

double m = M_PI/(2*(pot_max-pot_min));
double offset = M_PI/4-m*pot_max;

// Global flag to control when the ISR prints data
bool enableIsrPrinting = false;

int sign = -1;
// NEW: Array to store the results of each stiction test
const int MAX_RESULTS = 20;
float stiction_results[MAX_RESULTS];
int result_count = 0;

double map_potentiometer(int val){
  double radians = m*val+offset;
  return radians;
}

void runStictionTest() {
  // --- Test Parameters ---
  const float start_val = sign*0.50;                   // Starting voltage for the test
  const float increment_val = sign*0.01;                // Voltage step
  const int   readings_per_step = 40;               // How many angle readings per voltage
  const int   delay_between_readings_ms = 100;     // Wait time between readings
  const float change_val_deg = 3.0;                // Min degrees of movement to detect
  const int   required_consecutive_readings = 3;   // Readings needed to confirm movement
  const float max_test_volt = 2.0;                 // Safety limit for the test voltage
              

  Serial.println("\n===== Starting Automated Stiction Test =====");
  enableIsrPrinting = false; // Disable ISR printing for clean, plottable output

  // --- POSITIVE DIRECTION TEST ---
  Serial.println("\n--- Testing stiction ---");
  Serial.println("(Voltage, Angle)"); // Header for CSV data
  setMotorVoltage(0.0f);
  delay(1000); // Wait for motor to settle

  // Average 5 readings to get a stable initial position
  Serial.println("Averaging initial position...");
  double initial_angle_sum = 0;
  const int readings_for_average = 5;
  for (int i = 0; i < readings_for_average; i++) {
    initial_angle_sum += map_potentiometer(analogRead(MOT_PIN)) * 180.0 / M_PI;
    delay(20); // Small delay between readings for stability
  }
  double init_deg = initial_angle_sum / readings_for_average;
  Serial.print("Initial Position set to (deg): ");
  Serial.println(init_deg, 2);

  int consecutive_detections = 0; // Counter for consecutive readings outside the noise band
  
  // Outer loop: Increment the voltage, starting from start_val
  for (float v = start_val; v <= max_test_volt; v += increment_val) {
    setMotorVoltage(v);
    
    // Inner loop: Take multiple readings at the current voltage
    for (int i = 0; i < readings_per_step; i++) {
      delay(delay_between_readings_ms);
      double current_deg = map_potentiometer(analogRead(MOT_PIN)) * 180.0 / M_PI;
      
      // Print the data pair for plotting
      Serial.print("(");
      Serial.print(v, 2);
      Serial.print(", ");
      Serial.print(current_deg, 2);
      Serial.println(")");

      // Check if the current reading is outside the initial noise band
      if (abs(current_deg - init_deg) > change_val_deg) {
        consecutive_detections++;
      } else {
        consecutive_detections = 0;
      }

      // Check if we have reached the required number of consecutive detections
      if (consecutive_detections >= required_consecutive_readings) {
        Serial.print("--> Movement confirmed at: ");
        Serial.print(v, 2);
        Serial.println(" V");
        
        // NEW: Add result to history array and print all results
        if (result_count < MAX_RESULTS) {
          stiction_results[result_count] = v;
          result_count++;
        }
        
        Serial.println("\n--- Stiction Test History ---");
        for (int j = 0; j < result_count; j++) {
          Serial.print("Test ");
          Serial.print(j + 1);
          Serial.print(": ");
          Serial.println(stiction_results[j], 2);
        }
        Serial.println("---------------------------");

        // Stop the motor and exit the function immediately
        setMotorVoltage(0.0f);
        Serial.println("\nTest stopped. Send 's' to run again.");
        return; 
      }
    }
  }

  // --- CLEANUP ---
  setMotorVoltage(0.0f);
  Serial.println("\n===== Test Finished: Max voltage reached without movement =====");
}
  
// ================== Setup ==================
void setup() {
  pinMode(A5, OUTPUT);
  Serial.begin(115200);
  delay(300);
  
  geeWhizBegin();                 
  set_control_interval_ms(100);
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");
  Serial.println("Send 's' to start the automated stiction test.");
}

// ================== Loop ==================
void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 's') {
      runStictionTest();
    }
    while(Serial.available() > 0) {
      Serial.read();
    }
  }
}

// ================== Control ISR ==================
void interval_control_code(void) {
  if (enableIsrPrinting) {
    int motor_raw = analogRead(MOT_PIN);
    double motor_angle_deg = map_potentiometer(motor_raw) * 180.0 / M_PI;
    Serial.print("Current Angle (deg): ");
    Serial.println(motor_angle_deg);
  }
}