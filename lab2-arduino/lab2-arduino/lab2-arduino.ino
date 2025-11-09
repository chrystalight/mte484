#include <Arduino.h>
#include "geeWhiz.h"
#include <cppQueue.h>


//for STATION 13: 
//#define N_ZEROS 26
//#define N_POLES 27
//const int STATION_NUM = 13; 

//for STATION 12:
#define N_ZEROS 24
#define N_POLES 25
const int STATION_NUM = 12; 

// ========== Pins ==========
const int MOT_PIN = A0;   // motor angle sensor
const int BAL_PIN = A1;   // ball position sensor

// ========== System Parameters ==========
const int T = 4;                                  // Sampling time in MS
const float stiction_offset_neg = -0.54;
const float stiction_offset_pos = 0.08;
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
  double u_actual;
  double ball_raw;
};

volatile LogBook new_data;
volatile bool data_alarm = false;
void printLogBook(LogBook data){
    Serial.print(data.time_ms); //time in ms since we started trial
    Serial.print(",");
    Serial.print(data.ref, 5);  //reference angle
    Serial.print(",");
    Serial.print(data.constrained_ref, 5); //reference angle after saturator
    Serial.print(",");
    Serial.print(data.current_angle, 5); //current calculated angle
    Serial.print(",");
    Serial.print(data.trial_num); //index for autostepping
    Serial.print(",");
    Serial.print(data.trial_value); //initial reference angle again for some reason
    Serial.print(",");
    Serial.print(data.u_actual, 5); //controller output
    Serial.print(",");
    Serial.println(data.ball_raw); //ball position (raw)
}


//========== For Controller Implementation ==========
#define	IMPLEMENTATION FIFO

cppQueue u_queue(sizeof(double), N_POLES, IMPLEMENTATION, true); //true to enable overwriting
cppQueue e_queue(sizeof(double), N_ZEROS, IMPLEMENTATION, true);
const double* a_coeffs;
const double* b_coeffs;

const double a_coeffs_13[26] = {
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

const double b_coeffs_13[27] = {
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

const double a_coeffs_12[24] = {
  -3.202514621083537,
   4.850224482639266,
  -2.416024408545324,
   0.212847430192880,
   0.028989501923688,
  -0.060439898641832,
  -0.034253271246125,
  -0.034632169652408,
  -0.034064273291039,
  -0.042682683693496,
  -0.035488589185325,
  -0.053165135000975,
  -0.017838740264721,
  -0.037876854795247,
  -0.040532039423594,
  -0.037821079266218,
  -0.040073131126908,
  -0.035152836361889,
   0.435700007849979,
  -0.972487391491639,
   0.324433819811870,
   0.527882343393414,
  -0.931902585643497,
   0.654129594424024,
}; // coefficients on the numerator of D

const double b_coeffs_12[25] = {
   1.000000000000000,
  -1.514373454883820,
   0.751747880713601,
  -0.069707647860514,
  -0.011286893018807,
   0.015936501638972,
   0.007361160757791,
   0.007004369403117,
   0.006099674082183,
   0.008844212906579,
   0.006217724944567,
   0.010522698373411,
   0.000356225471464,
   0.005381233935217,
   0.005445719394046,
   0.004384858216212,
   0.003883479400783,
   0.003446103609769,
   0.003005770506565,
   0.002562775625260,
   0.002535817206579,
   0.002019775457137,
   0.001126440068917,
   0.001012971739118,
   0.000561335528298,
}; //coefficients on the denominator of D
// ========== For Filtering ===============
volatile int g_latestBalValue;
const int MIN_BAL_READING = 300;
const int MAX_BAL_READING = 900;
const double FILTER_ALPHA_MOTOR = 0.2; 
const double FILTER_ALPHA_BALL = 0.6;
double filtered_mot_raw;
double filtered_bal_raw;

// ========== For testing /input generation ==========

// --- Sin Parameters ---
float frequency_hz;                         // Default for sine wave mode
const float amplitude_rad = M_PI / 8.0;
float step_magnitude_rad;                   // Default for step input mode
float open_loop_voltage;                    // Default for open-loop mode
// --- Autostepping --- 
float rad_mag[] = {0, -0.7, 0, 0.7, 0, -0.7, 0, 0.7};
int num_steps = sizeof(rad_mag) / sizeof(rad_mag[0]);
int step_index = 0;
bool autostep = true;

// =============== State setup ===============
enum ControlMode{
  STEP_INPUT = 1,
  SINE_INPUT = 2,
  CALIBRATE_BALL = 3,
  ISR_TIMING_TEST = 4
};

enum ProgramState { 
  WAIT_FOR_INPUT = 1, 
  RUNNING_TEST = 2, 
  TEST_COMPLETE = 3
};

int input_mode = STEP_INPUT;

volatile int currentState = WAIT_FOR_INPUT;
volatile int i = 0;
volatile double target_angle = -1;
double trial_value = -1; //mode-dependent, eg. step magnitude
volatile int trial_num = 1; // 1-indexed because matlab

//=================== Potentiometer Calibrations ====
// // station 13
// const int motor_pot_min = 565;
// const int motor_pot_max = 455;

// station 12
const int motor_pot_min = 443;
const int motor_pot_max = 336;

const double motor_pot_slope = M_PI / (2.0 * (motor_pot_max - motor_pot_min));
const double motor_pot_offset = M_PI / 4.0 - motor_pot_slope * motor_pot_max;


double ball_pos_1 = 0.1; //meters
double ball_pos_2 = 0.25; //meters
double ball_reading_1 = 0; //sensor output @ 0.1 m
double ball_reading_2 = 0; //sensor output @ 0.25 m


double ball_m = (ball_pos_2 - ball_pos_1) / (ball_reading_2 - ball_reading_1);
double ball_b = ball_pos_1 - ball_m * ball_reading_1;


double map_potentiometer(int val) {
  double radians = motor_pot_slope * val + motor_pot_offset;
  return radians;
}

double map_ball_sensor(int val){
  double meters = ball_m * val + ball_b;
  return meters;
}

// ================== HELPER FUNCTIONS ====

void startTest(float userInput){
      Serial.println("==============================================================================");
      if  (input_mode == STEP_INPUT) {
        if(autostep){
          target_angle = rad_mag[step_index];
          trial_value = target_angle;
          Serial.print("NEW TEST STARTED -> REFERENCE ANGLE: ");
          Serial.print(target_angle, 4);
          Serial.println(" rad");
        }
        else{
          step_magnitude_rad = userInput;
          trial_value = step_magnitude_rad;
          double current_angle = map_potentiometer(analogRead(MOT_PIN)); 
          target_angle = current_angle + step_magnitude_rad;
          Serial.print("NEW TEST STARTED -> STEP MAGNITUDE: ");
          Serial.print(step_magnitude_rad, 4);
          Serial.println(" rad");
        }
      } 
      else if (input_mode == SINE_INPUT) {
        frequency_hz = userInput;
        trial_value = frequency_hz;
        Serial.print("NEW TEST STARTED -> SINE WAVE FREQUENCY: ");
        Serial.print(frequency_hz, 2);
        Serial.println(" Hz");
      }
      else if(input_mode == CALIBRATE_BALL){
        target_angle = 0;
      }

      Serial.println("==============================================================================");
      Serial.println("Time (ms),Original Ref (rad),Final Ref (rad),Angle (rad),TrialNum,TrialValue,U_Actual (V),Ball");
      
      i = 0;
      currentState = RUNNING_TEST;
}

void endTest(){
    Serial.println("==============================================================================");
    Serial.println("Test complete.");
    trial_num += 1;

    currentState = WAIT_FOR_INPUT; //so any interrupts here are just passed

    
    if (input_mode == STEP_INPUT){
      if(autostep){
        step_index++;
        if (step_index < num_steps) {
            Serial.println("Please wait for the next step to occur.");
            startTest(-1);
        } else {
            Serial.println("\n You're finished!!! Returning to normal operation now");
            autostep = false;
        }
      }
        Serial.println("\nPlease enter a new step magnitude to run another test:");
    } else if (input_mode == SINE_INPUT) {
        Serial.println("\nPlease enter a new frequency to run another test:");
    }


}

void fillQueueWithZero(cppQueue& q, int size) {
  double zero = 0.0;
  for (int i = 0; i < size; i++) {
    q.push(&zero); 
  }
}

// ================== Setup ==================
void setup() {
  pinMode(A5, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  geeWhizBegin();
  set_control_interval_ms(T);
  setMotorVoltage(0);
  g_latestBalValue = analogRead(BAL_PIN);

  if(STATION_NUM == 12){
    a_coeffs = a_coeffs_12;
    b_coeffs = b_coeffs_12;
  }
  else if(STATION_NUM == 13){
    a_coeffs = a_coeffs_13;
    b_coeffs = b_coeffs_13;
  }

  //prime variables so that we don't get wonky results 
  fillQueueWithZero(u_queue, N_POLES);
  fillQueueWithZero(e_queue, N_ZEROS);
  filtered_mot_raw = analogRead(MOT_PIN);
  filtered_bal_raw = analogRead(BAL_PIN);

  Serial.println("==================================================");

  switch(input_mode){

    case STEP_INPUT: 
      Serial.println("Mode: STEP INPUT");
      if(autostep){
        Serial.println("(AUTOMATED)");
        Serial.println("\nPress any character and Enter to start the trial:");
      }
      else{
        Serial.println("\nPlease enter a STEP MAGNITUDE (in radians) and press Enter:");
      }
      break;

    case SINE_INPUT:
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
    // Use memcpy to copy the value of 'a' into 'b'
    memcpy(&curr_data, (void*)&new_data, sizeof(LogBook));    
    data_alarm = false;
    interrupts();
    printLogBook(curr_data);
  }
  else{
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

  //oversample at whatever rate this loop runs at, protect the value from being read while its being written
  int newSample = analogRead(BAL_PIN);
  if (newSample > MIN_BAL_READING && newSample < MAX_BAL_READING) {
    float newAverage = (FILTER_ALPHA_BALL * (float)newSample) + ((1.0 - FILTER_ALPHA_BALL) * g_latestBalValue);
    noInterrupts();
    g_latestBalValue = newAverage;
    interrupts();
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

  int motor_raw = analogRead(MOT_PIN);
  int filtered_bal_raw = g_latestBalValue;
  // Apply Exponential Moving Average (EMA) filter
  filtered_mot_raw = (FILTER_ALPHA_MOTOR * motor_raw) + ((1.0 - FILTER_ALPHA_MOTOR) * filtered_mot_raw); //filtered_mot_raw holds the most recent value
  // ---- Read sensor ----
  double current_angle = map_potentiometer(filtered_mot_raw);
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

    //to get error at current time, you need to check index of queue-size - 1. index[0] --> oldest error

    //first subtract: b coefficients, from 1 to N_POLES --> b1 * u [k - 1]
    for(int j = 1; j<N_POLES; j++){
      u_queue.peekIdx(&peeker, N_POLES-j);
      u_total -= b_coeffs[j]*peeker;

      //example: at j = 1
      //u_queue.peekIdx(&peeker, N_POLES-j); --> this gives the 'last' value of u_queue, which is the most recently saved U value, u[k-1]
      //u_total -= b_coeffs[j]*peeker; --> this gets b1, giving b1*u[k-1]

      //example: at j = n_poles-1
      //u_queue.peekIdx(&peeker, N_POLES-j); --> this gives u_queue[0], , which is oldest historical value, u[k-27] in this case
      //u_total -= b_coeffs[j]*peeker; --> this gets b[26], the last b coefficient, giving b[26]*u[k-26]
    }

    for(int j = 0; j<N_ZEROS; j++){
      e_queue.peekIdx(&peeker, N_ZEROS-j-1);
      u_total += a_coeffs[j]*peeker;
      //we want to do a[j] times e[k-j] e.g. starting with a[0]*e[k]
      //e[k-j] is counted backwards starting from N_ZEROS-1 --> e[k-j] is at index (N_ZEROS-1-j)
    }

    double U = u_total/b_coeffs[0];
    u_queue.push(&U);
    

    //stiction comp:
    double U_comped = U;
    if (abs(U) > 0.01) {
      U_comped = U + (U > 0 ? stiction_offset_pos : stiction_offset_neg);
    }

    if (input_mode == ISR_TIMING_TEST) {
      // perform all the calculations above, 
      // but hardcode the output voltage to alternate between 0 and 5
      U_comped = (i % 2) * 5;
    }
    setMotorVoltage(U_comped);
  
  // ---- log data ----
    new_data.time_ms = T * i;
    new_data.ref = ref;
    new_data.constrained_ref = constrained_ref;
    new_data.current_angle = current_angle;
    new_data.trial_num = trial_num;
    new_data.trial_value = trial_value;
    new_data.u_actual = U_comped;
    new_data.ball_raw = filtered_bal_raw;

    data_alarm = true;

    i++;
}


