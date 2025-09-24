#include <Arduino.h>
#include "geeWhiz.h"

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
  
// ================== Setup ==================
void setup() {

  pinMode(A5, OUTPUT);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);
  delay(1000);
  calibrate_potentiometer(); //comment out this line to use the defaults


  geeWhizBegin();                 
  set_control_interval_ms(1000); // 100 ms loop
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");

}

// ================== Loop ==================
void loop() {
  
}


// ================== Control ISR ==================
void interval_control_code(void) {
  // ---- Read sensors ----
  int motor = analogRead(MOT_PIN);
  int ball  = analogRead(BAL_PIN);

  digitalWrite(A5,HIGH);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  //Serial.print(ball);
  //Serial.print(",");
  Serial.print(motor);
  Serial.print(", ");
  Serial.print(map_potentiometer(motor));
  Serial.print(", ");
  Serial.print(m);
  Serial.print(" ");
  Serial.println(offset);
  digitalWrite(A5,LOW);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  

}