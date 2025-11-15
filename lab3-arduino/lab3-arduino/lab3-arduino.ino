#include <Arduino.h>
#include "geeWhiz.h"
#include <cppQueue.h>

// ========== TODO: POPULATE OUTER LOOP CONTROLLER =========
//for STATION 13: 
#define N_ZEROS_INNER 26
#define N_POLES_INNER 27

const int STATION_NUM = 13; 

#define N_ZEROS_OUTER 27
#define N_POLES_OUTER 28

//for STATION 12:
//#define N_ZEROS_INNER 24
//#define N_POLES_INNER 25
//const int STATION_NUM = 12; 

// ========== Pins ==========
const int MOT_PIN = A0;   // motor angle sensor
const int BAL_PIN = A1;   // ball position sensor

// ========== System Parameters ==========
const int T_INNER = 4;   // Sampling time in MS
const int T_OUTER = 240;   
const int OUTER_DIV = T_OUTER/T_INNER; //T_outer/T_innter
static int outer_div_ctr = 0;     


const float stiction_offset_neg = -0.54;
const float stiction_offset_pos = 0.08;
const int max_T = 15000;                          // Test duration in MS
const float saturation_limit = 0.7;         // The limit for the reference signal in radians

// ========== For Logging =============
struct LogBook {
long time_ms;
double y_ref;         // Outer loop reference (meters)
double y_current;     // Current ball position (meters)
double theta_ref_raw; // Outer loop output (radians)
double theta_ref_sat; // Saturated inner loop reference (radians)
double theta_current; // Current motor angle (radians)
int trial_num;
double u_actual;      // Final motor voltage (Volts)
};

volatile LogBook new_data;
volatile bool data_alarm = false;
void printLogBook(LogBook data){
Serial.print(data.time_ms); //time in ms
Serial.print(",");
Serial.print(data.y_ref, 5);    //y_ref
Serial.print(",");
Serial.print(data.y_current, 5); //y_current
Serial.print(",");
Serial.print(data.theta_ref_raw, 5); //theta_ref from outer_ctrl
Serial.print(",");
Serial.print(data.theta_ref_sat, 5); //theta_ref after saturator
Serial.print(",");
Serial.print(data.theta_current, 5); //theta_current
Serial.print(",");
Serial.print(data.trial_num); //index for autostepping
Serial.print(",");
Serial.println(data.u_actual, 5); //controller output
}


//========== For Controller Implementation ==========
#define	IMPLEMENTATION FIFO
// --- Inner Loop (D1) Queues ---
cppQueue u_queue_inner(sizeof(double), N_POLES_INNER, IMPLEMENTATION, true);
cppQueue e_queue_inner(sizeof(double), N_ZEROS_INNER, IMPLEMENTATION, true);
// --- Outer Loop (D2) Queues ---
cppQueue u_queue_outer(sizeof(double), N_POLES_OUTER, IMPLEMENTATION, true);
cppQueue e_queue_outer(sizeof(double), N_ZEROS_OUTER, IMPLEMENTATION, true);

// Station 13
const double a_coeffs_inner[26] = {
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

const double b_coeffs_inner[27] = {
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

// not sure if these are station-specific... pretty sure they aren't?
const double a_coeffs_outer[N_ZEROS_OUTER] = {
  -4.166665421620564,
  -1.028140433768777,
   1.230834085913079,
   3.128888986876133,
   1.940257030121576,
   0.843718317973515,
   0.023445901918542,
   0.044693033799688,
   0.076610857235621,
   0.013678545633188,
  -0.227974073826002,
  -0.461239299828000,
  -0.648910517149018,
  -0.796753124959115,
  -0.462512644689705,
  -0.045955198965863,
   0.404706405696049,
   0.698095670549242,
   0.614455840198516,
   0.285618177446519,
   0.111583718043341,
  -0.761085790758492,
  -0.732496799443088,
  -2.345917540318282,
   0.032825759036112,
   1.056486801259040,
   1.171751988946379
}; 

const double b_coeffs_outer[N_POLES_OUTER] = {
   1.000000000000000,
   0.246754474847157,
   0.004961809880291,
  -0.451809179774356,
  -0.272132742819161,
  -0.221089663459667,
  -0.115213273738796,
  -0.150611557407302,
  -0.144187965432549,
  -0.133827720823898,
  -0.089820453041181,
  -0.053992283828072,
  -0.023651858000354,
  -0.004221446676769,
   0.009615206109704,
   0.014875152882891,
   0.012133750884051,
   0.005267269615967,
   0.000689372816615,
   0.003659843234899,
   0.015900253738757,
   0.034495436910785,
   0.055932621572818,
   0.072806452774950,
   0.079166123246922,
   0.063836130557479,
   0.032209001793042,
   0.008255309315977
};

// Station 12
/*const double a_coeffs_inner[24] = {
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

const double b_coeffs_inner[25] = {
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
*/


// ========== For Filtering ===============
volatile int g_latestBalValue;
const int MIN_BAL_READING = 300;
const int MAX_BAL_READING = 900;
const double FILTER_ALPHA_MOTOR = 0.2; 
const double FILTER_ALPHA_BALL = 0.6;
double filtered_mot_raw;
double filtered_bal_raw;

// ========== For testing /input generation ==========
// --- Autostepping --- 
float pos_mag[] = {0.10, 0.25, 0.10, 0.25, 0.10, 0.25, 0.10, 0.25, 0.10};
int num_steps = sizeof(pos_mag) / sizeof(pos_mag[0]);
int step_index = 0;
bool autostep = true;

// =============== State setup ===============
enum ControlMode{
  STEP_INPUT = 1,
  CALIBRATE_BALL = 2,
  ISR_TIMING_TEST = 3
};

enum ProgramState { 
  WAIT_FOR_INPUT = 1, 
  RUNNING_TEST = 2, 
  TEST_COMPLETE = 3
};

int input_mode = STEP_INPUT;

volatile int currentState = WAIT_FOR_INPUT;
volatile int i = 0;
volatile double g_target_pos = 0.1;
volatile double g_theta_ref_from_outer_loop = 0.0; // Global var to hold outer loop output
double trial_value = -1; //mode-dependent, eg. step magnitude
volatile int trial_num = 1; // 1-indexed because matlab

//=================== Potentiometer Calibrations ====
// // station 13
// const int motor_pot_min = 565;
// const int motor_pot_max = 455;

// station 12
const int motor_pot_min = 443;
const int motor_pot_max = 336;

const double motor_pot_slope = PI / (2.0 * (motor_pot_max - motor_pot_min));
const double motor_pot_offset = PI / 4.0 - motor_pot_slope * motor_pot_max;


double ball_pos_1 = 0.1; //meters
double ball_pos_2 = 0.25; //meters
//STATION 12 VALUES
double ball_reading_1 = 395; //sensor output @ 0.1 m
double ball_reading_2 = 545; //sensor output @ 0.25 m
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
   if (input_mode == STEP_INPUT) {
    if(autostep){
      g_target_pos = pos_mag[step_index];
      Serial.print("NEW TEST STARTED -> REFERENCE POSITION: ");
      Serial.print(g_target_pos, 4);
      Serial.println(" m");
      }
    else{
        g_target_pos = userInput;
        Serial.print("NEW TEST STARTED -> MANUAL POSITION: ");
        Serial.print(g_target_pos, 4);
        Serial.println(" m");
        }
        } 
    else if(input_mode == CALIBRATE_BALL){
        g_target_pos = 0.1; // just hold at 0.1m
    }

    Serial.println("==============================================================================");
    Serial.println("Time(ms),y_ref(m),y_curr(m),theta_ref_raw(r),theta_ref_sat(r),theta_curr(r),Trial,V_motor(V)");

    i = 0;
    outer_div_ctr = 0; // Reset outer loop counter
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
      else{
        Serial.println("\nPlease enter a new step magnitude to run another test:");
      }
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
  set_control_interval_ms(T_INNER);
  setMotorVoltage(0);
  g_latestBalValue = analogRead(BAL_PIN);

  //if(STATION_NUM == 12){
  //  a_coeffs_inner = a_coeffs_12_inner;
  //  b_coeffs_inner = b_coeffs_12_inner;
  //}

  //prime variables so that we don't get wonky results 
  fillQueueWithZero(u_queue_inner, N_POLES_INNER);
  fillQueueWithZero(e_queue_inner, N_ZEROS_INNER);
  fillQueueWithZero(u_queue_outer, N_POLES_OUTER);
  fillQueueWithZero(e_queue_outer, N_ZEROS_OUTER);
  filtered_mot_raw = analogRead(MOT_PIN);
  filtered_bal_raw = analogRead(BAL_PIN);

  Serial.println("==================================================");

  switch(input_mode){
  case STEP_INPUT: 
    Serial.println("Mode: STEP INPUT (Ball Position)");
    if(autostep){
      Serial.println("(AUTOMATED LAB 3 STEPS)");
      Serial.println("\nPress any character and Enter to start the trial:");
    }
    else{
      Serial.println("\nPlease enter a STEP POSITION (in meters) and press Enter:");
      }
      break;
    }
    Serial.println("==================================================");
  }
// ================== Loop ==================
void loop() {

  if (data_alarm){
    LogBook curr_data;
    noInterrupts(); //turn off interrupts to avoid race condition [isr and loop changing variable at same time]
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

/**
 * @brief Calculates the desired motor angle (theta_ref) based on ball position error.
 * @param y_ref The target ball position in meters.
 * @param y_current The current ball position in meters.
 * @return The calculated reference angle (theta_ref) in radians.
*/

double outer_ctrl(double y_ref, double y_current) {
 
  double error = y_ref - y_current;
  e_queue_outer.push(&error); // e[k] is at index N_ZEROS_OUTER - 1
  
  double u_total = 0.0;
  double peeker;

  // Denominator part: -b1*u[k-1] - b2*u[k-2] ...
  // Loop starts at j=1. u[k-1] is at index N_POLES_OUTER - 1
  // u[k-j] is at index N_POLES_OUTER - j
  for(int j = 1; j < N_POLES_OUTER; j++){
    u_queue_outer.peekIdx(&peeker, N_POLES_OUTER - j); // Get u[k-j]
    u_total -= b_coeffs_outer[j] * peeker;
  }

  // Numerator part: a0*e[k] + a1*e[k-1] ...
  // Loop starts at j=0. e[k] is at index N_ZEROS_OUTER - 1
  // e[k-j] is at index N_ZEROS_OUTER - 1 - j
  for(int j = 0; j < N_ZEROS_OUTER; j++){
    e_queue_outer.peekIdx(&peeker, N_ZEROS_OUTER - 1 - j); // Get e[k-j]
    u_total += a_coeffs_outer[j] * peeker;
  }

  double u_new = u_total / b_coeffs_outer[0]; // Assume b0 is 1, but good practice
  u_queue_outer.push(&u_new);
  return u_new;
}


/**
 * @brief Calculates the required motor voltage (U) based on motor angle error.
 * @param theta_ref The target motor angle in radians (from outer loop).
 * @param theta_current The current motor angle in radians.
 * @return The calculated motor voltage (U) in Volts.
 */

double inner_ctrl(double theta_ref, double theta_current) {
  double error = theta_ref - theta_current;
  e_queue_inner.push(&error); // e[k] is at index N_ZEROS_OUTER - 1
  
  double u_total = 0.0;
  double peeker;

  //Denominator part:-b1*u[k-1]-b2*u[k-2]
  // Loop starts at j=1. u[k-1] is at index N_POLES_OUTER - 1
  // u[k-j] is at index N_POLES_OUTER - j
  for(int j = 1; j < N_POLES_INNER; j++){
    u_queue_inner.peekIdx(&peeker, N_POLES_INNER - j); // Get u[k-j]
    u_total -= b_coeffs_inner[j] * peeker;
  }

  // Numerator part: a0*e[k] + a1*e[k-1] ...
  // Loop starts at j=0. e[k] is at index N_ZEROS_OUTER - 1
  // e[k-j] is at index N_ZEROS_OUTER - 1 - j
  for(int j = 0; j < N_ZEROS_INNER; j++){
    e_queue_inner.peekIdx(&peeker, N_ZEROS_INNER - 1 - j); // Get e[k-j]
    u_total += a_coeffs_inner[j] * peeker;
  }

  double u_new = u_total / b_coeffs_outer[0]; // Assume b0 is 1, but good practice
  u_queue_inner.push(&u_new);
  return u_new;
}

// ================== Control ISR ==================
void interval_control_code(void) {
    if (currentState != RUNNING_TEST) {
      return;
    }

    long current_time_ms = T_INNER * i;
    
    if (current_time_ms >= max_T) {
      //test is over!
      setMotorVoltage(0);
      currentState = TEST_COMPLETE;
      return;
    }

    int motor_raw = analogRead(MOT_PIN);
    int filtered_bal_raw = g_latestBalValue; //get oversampled value

    // Apply Exponential Moving Average (EMA) filter
    filtered_mot_raw = (FILTER_ALPHA_MOTOR * motor_raw) + ((1.0 - FILTER_ALPHA_MOTOR) * filtered_mot_raw); //filtered_mot_raw holds the most recent value
    // ---- Read sensor ----
    double current_angle = map_potentiometer(filtered_mot_raw);
    double current_y = map_ball_sensor(filtered_bal_raw);

    // ---- Outer Loop Controller (Runs at slower rate T_OUTER) ----

    if (outer_div_ctr == 0) {
      g_theta_ref_from_outer_loop = outer_ctrl(g_target_pos, current_y);
    }

    outer_div_ctr = (outer_div_ctr + 1) % OUTER_DIV; //increment counter and wrap around

    // ---- Inner Loop Controller (Runs at faster rate T_INNER) ----

    //Saturate the reference angle (uses the value from outer loop, which is held constant between outer loop ticks)
    double constrained_theta_ref = constrain(g_theta_ref_from_outer_loop, -saturation_limit, saturation_limit);

    // Calculate motor voltage from inner loop
    double U = inner_ctrl(constrained_theta_ref, current_angle);

    double U_comped = U;
      if (abs(U) > 0.01) { // Apply compensation if voltage is not tiny
      U_comped = U + (U > 0 ? stiction_offset_pos : stiction_offset_neg);
    }

    setMotorVoltage(U_comped);  
  // ---- log data ----
    new_data.time_ms = current_time_ms;
    new_data.y_ref = g_target_pos;
    new_data.y_current = current_y;
    new_data.theta_ref_raw = g_theta_ref_from_outer_loop;
    new_data.theta_ref_sat = constrained_theta_ref;
    new_data.theta_current = current_angle;
    new_data.trial_num = trial_num;
    new_data.u_actual = U_comped;

    data_alarm = true;
    i++;
}



