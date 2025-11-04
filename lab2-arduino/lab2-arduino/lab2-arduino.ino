#include <Arduino.h>
#include "geeWhiz.h"
#include <cppQueue.h>

#define n_zeroes 26
#define n_poles 27

// ========== Pins ==========
const int MOT_PIN = A0;   // motor angle sensor
const int BAL_PIN = A1;   // ball position sensor

// ========== System Parameters ==========
const int T = 5;                                  // Sampling time in MS
const float stiction_offset_neg = -0.202;
const float stiction_offset_pos = 0.236;
const int max_T = 5000;                          // Test duration in MS
const float saturation_limit = 0.7;         // The limit for the reference signal in radians

// ========== For Logging =============
struct LogBook {
  //we probably do not need all of these values but i can figure it out
  long time_ms;
  double ref;             
  double constrained_ref; 
  double current_angle;
  int trial_num;
  double trial_value;
  int motor_raw;
};
volatile LogBook new_data;
volatile bool data_alarm = false;
void printLogBook(LogBook data){
    Serial.print(data.time_ms);
    Serial.print(",");
    Serial.print(data.ref, 5); 
    Serial.print(",");
    Serial.print(data.constrained_ref, 5);
    Serial.print(",");
    Serial.print(data.current_angle, 5);
    Serial.print(",");
    Serial.print(data.trial_num);
    Serial.print(",");
    Serial.print(data.trial_value);
    Serial.print(",");
    Serial.println(data.motor_raw);
}


//========== For Controller Implementation ==========
#define	IMPLEMENTATION FIFO
cppQueue u_queue(sizeof(double), n_poles, IMPLEMENTATION);
cppQueue e_queue(sizeof(double), n_zeroes, IMPLEMENTATION);

const double a_coeffs[n_zeroes] = {
  -4.014671308410470,
  6.595993799639460,
  -3.837648018627513,
  0.708976866805470,
  -0.025995388538642,
  -0.076514941608012,
  -0.032012691794383,
  -0.037463474376800,
  -0.035306296626180,
  -0.048345742227537,
  -0.038158308502171,
  -0.058766459874832,
  -0.013873291178805,
  -0.043106553671307,
  -0.044733692793647,
  -0.039419151035545,
  -0.038595190582891,
  -0.033039877980923,
  0.028878298445661,
  -0.197134924415200,
  0.053894813684000,
  0.215191055728711,
  -0.810608196173737,
  0.833830782048386,
  -0.088481927716491,
  0.004242677516370,
}; // coefficients on the numerator of D

const double b_coeffs[n_poles] = {
  1.000000000000000,
  -1.642984519812282,
  0.958321968432410,
  -0.183217773360566,
  0.006068215325551,
  0.016793826989395,
  0.005376007191384,
  0.006431007500784,
  0.005490457115507,
  0.008359531301715,
  0.005382362473137,
  0.010070061195247,
  -0.000666309620383,
  0.005730533574389,
  0.005043448210347,
  0.004041500950617,
  0.003693869678733,
  0.003322130108701,
  0.002947770343532,
  0.002569995478323,
  0.002199729812247,
  0.002003577307391,
  0.001471797034653,
  0.000970219358741,
  0.001403748774129,
  -0.000165534783335,
  0.000008696897701,
}; //coefficients on the denominator of D

// ========== For testing /input generation ==========

// --- Sin Parameters ---
float frequency_hz;                         // Default for sine wave mode
const float amplitude_rad = M_PI / 8.0;
float step_magnitude_rad;                   // Default for step input mode
float open_loop_voltage;                    // Default for open-loop mode
// --- Autostepping --- 
float rad_mag[] = {M_PI/8, -M_PI/8, M_PI/4, -M_PI/4};
int num_steps = sizeof(rad_mag) / sizeof(rad_mag[0]);
int step_index = 0;
bool autostep = false;

// =============== State setup ===============
enum ControlMode{
  STEP_INPUT = 1,
  SINE_INPUT = 2
};

enum class ProgramState { 
  WAIT_FOR_INPUT, 
  RUNNING_TEST, 
  TEST_COMPLETE 
};

int input_mode = STEP_INPUT;

volatile ProgramState currentState = WAIT_FOR_INPUT;
volatile int i = 0;
volatile double target_angle = -1;
double trial_value = -1; //mode-dependent, eg. step magnitude
volatile int trial_num = 1; // 1-indexed because matlab

//=================== Potentiometer Calibration ====
int pot_min = 565;
int pot_max = 455;
double m = M_PI / (2.0 * (pot_max - pot_min));
double offset = M_PI / 4.0 - m * pot_max;

double map_potentiometer(int val) {
  double radians = m * val + offset;
  return radians;
}

// ================== HELPER FUNCTIONS ====

void startTest(float userInput){
      Serial.println("==============================================================================");
      if  (input_mode == STEP_INPUT) {
        if(autostep){
          step_magnitude_rad = rad_mag[step_index];
        }
        else{
          step_magnitude_rad = userInput;
        }
        trial_value = step_magnitude_rad;
        double current_angle = map_potentiometer(analogRead(MOT_PIN)); 
        target_angle = current_angle + step_magnitude_rad;

        Serial.print("NEW TEST STARTED -> STEP MAGNITUDE: ");
        Serial.print(step_magnitude_rad, 4);
        Serial.println(" rad");
      } else if (input_mode == SIN_INPUT) {
        frequency_hz = userInput;
        trial_value = frequency_hz;
        Serial.print("NEW TEST STARTED -> SINE WAVE FREQUENCY: ");
        Serial.print(frequency_hz, 2);
        Serial.println(" Hz");
      }

      Serial.println("==============================================================================");
      Serial.println("Time (ms),Original Ref (V or rad),Final Ref (V or rad),Angle (rad),TrialCount,TrialValue,Raw Sensor");
      
      i = 0;
      currentState = RUNNING_TEST;
}

void endTest(){
    Serial.println("==============================================================================");
    Serial.println("Test complete.");
    trial_num += 1;
    
    if (input_mode == STEP_INPUT) 
      if(autostep){
        step_index++;
        if (step_index < num_steps) {
            Serial.println("\nInput a character and enter to run the next one");
        } else {
            Serial.println("\n You're finished!!! Returning to normal operation now");
            autostep = false;
        }
      }
        Serial.println("\nPlease enter a new step magnitude to run another test:");
    } else if (input_mode == SIN_INPUT) {
        Serial.println("\nPlease enter a new frequency to run another test:");
    }

    currentState = WAIT_FOR_INPUT;

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

  switch(input_mode){

    case STEP_INPUT: 
      Serial.println("Mode: STEP INPUT");
      if(autostep){
        Serial.println("(AUTOMATED)");
        Serial.println("\nPress any character and Enter to start the first step trial:");
      }
      else{
        Serial.println("\nPlease enter a STEP MAGNITUDE (in radians) and press Enter:");
      }
      break;

    case SIN_INPUT:
      Serial.println("Mode: SINE WAVE TRACKING");
      Serial.println("\nPlease enter a SINE WAVE FREQUENCY (in Hz) and press Enter:");
      break;
  }

  Serial.println("==================================================");
}

// ================== Loop ==================
void loop() {

  if (data_alarm){
    LogBook curr_data;
    noInterrupts(); //turn off interrupts to avoid race condition [isr and loop changing variable at same time]
    curr_data = new_data;
    data_alarm = false;
    interrupts();
    printLogBook(curr_data);
  }

  switch(currentState){
    case RUNNING_TEST:
      break;
    case WAIT_FOR_INPUT: 
      if (Serial.available() > 0) {
        float userInput = Serial.parseFloat();
        while (Serial.available() > 0) { Serial.read(); }
        startTest(userInput);
      }
      break;

    case TEST_COMPLETE:
      endTest();
      break;  
  }
}

// ================== Control ISR ==================
void interval_control_code(void) {

  if (currentState != RUNNING_TEST) {
    return;
  }
  
  if (T * i >= max_T) {
    //test is over!
    setMotorVoltage(0);
    currentState = TEST_COMPLETE;
    return;
  }

  // ---- Read sensor ----
  int motor_raw = analogRead(MOT_PIN);
  double current_angle = map_potentiometer(motor_raw);

  // ---- Closed Loop Control Logic ----

  double ref;
  // ---- Control Logic based on Mode ----
    if (input_mode == STEP_INPUT) {
      ref = target_angle;
    } 
    else { // input_mode == 2, calculate sin wave
      float current_time_s = (float)(T * i) / 1000.0;
      ref = amplitude_rad * sin(2.0 * M_PI * frequency_hz * current_time_s);
    }

    double constrained_ref = constrain(ref, -saturation_limit, saturation_limit);
    
    // ---- Discrete Controler ----
    
    double error = constrained_ref - current_angle;
    e_queue.push(&error);
    double u_total = 0.0;
    double peeker;

    //CHECK QUEUE INDEX ORDER LOGIC IM NOT CONFIDENT 
    for(int j = 1; j<n_poles; j++){
      u_queue.peekIdx(&peeker, j);
      u_total -= b_coeffs[j]*peeker;
    }

    for(int j = 0; j<n_zeroes; j++){
      e_queue.peekIdx(&peeker, j);
      u_total += a_coeffs[j]*peeker;
    }

    double U = u_total/b_coeffs[0];
    u_queue.push(&U);
    

    //stiction comp:
    double U_comped = U;
    if (abs(U) > 0.01) {
      U_comped = U + (U > 0 ? stiction_offset_pos : stiction_offset_neg);
    }

    setMotorVoltage(U_comped);
  
  // ---- log data ----
    new_data.time_ms = T * i;
    new_data.ref = ref;
    new_data.constrained_ref = constrained_ref;
    new_data.current_angle = current_angle;
    new_data.trial_num = trial_num;
    new_data.trial_value = trial_value;
    new_data.motor_raw = motor_raw;

    data_alarm = true;

    i++;
}


