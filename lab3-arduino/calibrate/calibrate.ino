#include <Arduino.h>

// ========== Configuration ==========

// SET THIS VARIABLE to the sensor you want to calibrate:
// MOTOR or BALL
enum SensorToCalibrate { MOTOR, BALL };
const SensorToCalibrate calibrationMode = BALL;

// Define your sensor pins
const int MOT_PIN = A0; 
const int BAL_PIN = A1;

// ========== Filter & Timer Settings ==========
const double FILTER_ALPHA = 0.05; 
double filtered_value;
const long PRINT_INTERVAL = 500;
unsigned long previousPrintTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait
  }

  int initial_pin;
  String sensor_name;
  String instructions;

  if (calibrationMode == MOTOR) {
    initial_pin = MOT_PIN;
    sensor_name = "Motor Potentiometer";
    instructions = "Slowly move the MOTOR ARM to its MIN/MAX positions.";
  } else {
    initial_pin = BAL_PIN;
    sensor_name = "Ball Sensor";
    instructions = "Slowly move the BALL to its MIN/MAX positions (e.g., 0.1m and 0.25m).";
  }

  filtered_value = analogRead(initial_pin);

  Serial.println("========================================");
  Serial.print("Calibrating: ");
  Serial.println(sensor_name);
  Serial.println("========================================");
  Serial.println(instructions);
  Serial.println("\nFiltered ADC Reading:");
}

void loop() {
  int raw_reading;

  if (calibrationMode == MOTOR) {
    raw_reading = analogRead(MOT_PIN);
  } else {
    raw_reading = analogRead(BAL_PIN);
  }

  filtered_value = (FILTER_ALPHA * raw_reading) + ((1.0 - FILTER_ALPHA) * filtered_value);

  unsigned long currentTime = millis();
  
  if (currentTime - previousPrintTime >= PRINT_INTERVAL) {
    previousPrintTime = currentTime;
    Serial.println((int)filtered_value);
  }
}