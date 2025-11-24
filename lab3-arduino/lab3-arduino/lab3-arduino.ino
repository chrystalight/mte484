#include <Arduino.h>
#include "geeWhiz.h"
#include <cppQueue.h>

// ========== TODO: POPULATE OUTER LOOP CONTROLLER =========
//for STATION 13:

//#define N_ZEROS_INNER 26
//#define N_POLES_INNER 27

//for STATION 13 NEW VALS:
#define N_ZEROS_INNER 25
#define N_POLES_INNER 26

//working(?) integrator attempt on station 13
#define N_ZEROS_OUTER 88
#define N_POLES_OUTER 89

const int STATION_NUM = 13; 

//CTRL attempt 1, November 11, with 0.240s sampling time
//#define N_ZEROS_OUTER 27
//#define N_POLES_OUTER 28

//CTRL attempt with 0.400s sampling time
//#define N_ZEROS_OUTER 33
//#define N_POLES_OUTER 34

//attempt w/ 360ms sampling
// #define N_ZEROS_OUTER 24
// #define N_POLES_OUTER 25

//for STATION 12:
//#define N_ZEROS_INNER 24
//#define N_POLES_INNER 25
//const int STATION_NUM = 12; 

// ========== Pins ==========
const int MOT_PIN = A0;   // motor angle sensor
const int BAL_PIN = A1;   // ball position sensor

// ========== System Parameters ==========
const int T_INNER = 4;   // Sampling time in MS
//const int T_OUTER = 240;   //ATTEMPT 1
//const int T_OUTER = 400; //ATTEMPT 2
const int T_OUTER = 360;
const int OUTER_DIV = T_OUTER/T_INNER; //T_outer/T_innter
static int outer_div_ctr = 0;     


const float motor_stiction_offset_neg = -0.54;
const float motor_stiction_offset_pos = 0.08;
const float beam_stiction_offset_neg = -0.24;
const float beam_stiction_offset_pos = 0.23; //may need to change per-station
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

//old values 

/*
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

*/
//Station 13 NEW VALS


const double a_coeffs_inner[28] = {
  -4.026428254903294,
   6.615102778719837,
  -3.848776243664719,
   0.710992849531161,
  -0.026098713933281,
  -0.076766895707961,
  -0.032117673986351,
  -0.037584805262759,
  -0.035427277731845,
  -0.048539695165355,
  -0.038653052836755,
  -0.058875103918127,
  -0.013649601539569,
  -0.041897257773820,
  -0.047793085824696,
  -0.037449005293335,
  -0.040744391231529,
  -0.037140525695732,
  -0.019809504930329,
  -0.045348578533860,
  -0.004087483841310,
   0.017190840470117,
  -0.575752490856133,
   0.735821417199926,
  -0.066809735046565,
   0.001802185241076
};

const double b_coeffs_inner[27] = {
  1.000000000000000,
  -1.642943289402437,
   0.958543702586135,
  -0.183392300474898,
   0.006145152798565,
   0.016819238488557,
   0.005401785553243,
   0.006459883044782,
   0.005519424286866,
   0.008388977143563,
   0.005412315647343,
   0.010101272266585,
  -0.000634153487359,
   0.005763745149629,
   0.005075116996116,
   0.004073313010593,
   0.003730392539580,
   0.003355773034201,
   0.002984144560218,
   0.002607403901348,
   0.002259101756533,
   0.001905801372820,
   0.001540585933827,
   0.001198160184104,
   0.001312221613138,
  -0.000134326783812,
   0.000003811504057,
};


// Attempt 1 -- this goes with the sampling time of 0.240s, and is Nov11_final_controller
/*
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
*/

//Attempt 2 -- this goes with the sampling time of 0.640s, and is found in D_outer_640ms
/*
const double a_coeffs_outer[N_ZEROS_OUTER+1] = {
  -4.166654793589373,
  14.068365115716837,
 -19.321256368774783,
  15.513836801305700,
 -10.001154371353987,
   6.414544226977990,
  -4.220393642241348,
   2.399910369517359,
  -0.493257206150475,
  -0.326364970921651,
   0.188380771988638,
   0.363137259155915,
  -1.128505178690442,
   0.878784232587488,
  -0.559372244167000,
   0.124493398797685,
   2.072614593935477,
  -2.894194203257097,
   0.665577393827139,
   0.724946341335340,
  -0.172024917827348,
  -0.054582389373170,
  -0.494713274092662,
   1.082573501127513,
  -1.301566900197275,
   0.930904938317440,
  -0.327679350229599,
   0.034577564930512,
   0.004064702045338,
  -0.021578018859176,
   0.024327113040958,
  -0.012008900856693,
   0.004843711229285,
  -0.000575296485224
};

const double b_coeffs_outer[N_POLES_OUTER+1]{
  1.000000000000000,
  -3.181734568189126,
   4.546877301121634,
  -4.301077150936717,
   3.300588123971295,
  -2.304951882891881,
   1.536656252276429,
  -0.975980856430829,
   0.605012690384900,
  -0.375132359403927,
   0.215662871330010,
  -0.118178414279350,
   0.072017641744200,
  -0.025945702906005,
   0.027159470674311,
  -0.014963454186095,
  -0.002782407894749,
  -0.023823229108753,
   0.010388820754800,
   0.022121399684165,
  -0.005834077904810,
  -0.009076358581666,
  -0.000062914089976,
   0.003728420563016,
  -0.002789597797934,
   0.001959214781972,
   0.002546061866754,
  -0.004184651992275,
   0.000920918029411,
   0.000275983018225,
   0.000393565931044,
   0.000166078984097,
  -0.000006940521268,
   0.000061021960348,
  -0.000011257911414
};

// */
// //Attempt 3 -- this goes with T = 0.360s, no integrator
// const double a_coeffs_outer[N_ZEROS_OUTER+1] = {
//   -4.166666666512072,
//   -8.589736252233990,
//  -10.907581818230735,
//   -9.556799080620770,
//   -6.373786520195043,
//   -2.910562488552405,
//   -0.297136658630564,
//    1.260364829652884,
//    1.524420230596224,
//    0.890523451465900,
//    0.215265278027398,
//   -0.090934592604217,
//    0.332844412363310,
//    1.578427168250178,
//    3.107442116629638,
//    4.139334383344161,
//    3.900388775134775,
//    1.820607315855524,
//   -0.756490801244740,
//   -1.921459012447259,
//   -1.386499060518228,
//   -0.350843657941648,
//    0.494565600659102,
//    0.497352848118246,
//    0.206765460765875
// };
// const double b_coeffs_outer[N_POLES_OUTER+1]{
//    1.000000000000000,
//    2.285699882728168,
//    3.561435515964872,
//    4.278548895715130,
//    4.547295929678503,
//    4.437243672887059,
//    4.132107452186195,
//    3.692024999745091,
//    3.197796505539588,
//    2.700843796801034,
//    2.245749842241861,
//    1.832708938198274,
//    1.442325244112199,
//    1.054550126297804,
//    0.669723340809681,
//    0.315797129245805,
//    0.040185699124285,
//   -0.113769178467033,
//   -0.132620042483455,
//   -0.055346188615721,
//    0.039862473221423,
//    0.090171586140741,
//    0.084992922097811,
//    0.050666297242865,
//    0.018748264923441,
//    0.003468454512249
// };

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

//Integrator one - I think this works but wow it is big
const double a_coeffs_outer[N_ZEROS_OUTER+1] = {
  -0004.599939612326,
   0018.560539629846,
  -0049.605670833108,
   0109.937672014820,
  -0206.503898919810,
   0346.387051082800,
  -0530.559989129706,
   0751.383898049057,
  -0995.543219016513,
   1242.050408588654,
  -1466.323473059538,
   1643.349776244672,
  -1751.362008108880,
   1777.205929451415,
  -1717.058035477804,
   1577.587381145248,
  -1377.559455977028,
   1140.766446157397,
  -0894.103899332824,
   0662.997579140265,
  -0465.781471738660,
   0313.158461325313,
  -0206.912247149145,
   0141.213637495447,
  -0105.503756051409,
   0087.480845794026,
  -0075.779269497910,
   0062.357168282337,
  -0043.251367513152,
   0018.347039833746,
   0009.469876236407,
  -0036.176561161342,
   0058.054031623208,
  -0072.977701967657,
   0081.003188200850,
  -0084.015589558728,
   0085.035611386930,
  -0086.910312948162,
   0091.295251449612,
  -0098.076409905898,
   0105.395110402426,
  -0110.405770485061,
   0110.326783187071,
  -0103.435168072451,
   0089.654144793166,
  -0070.536121132407,
   0048.781600152784,
  -0027.468207566666,
   0009.289608369308,
   0003.867894554318,
  -0011.156795969464,
   0012.716349865979,
  -0009.443049974085,
   0002.774055234403,
   0005.577666553321,
  -0013.872752089269,
   0020.576404397602,
  -0024.577929674111,
   0025.351263609330,
  -0023.005324974408,
   0018.210611589300,
  -0011.999696294669,
   0005.499292630500,
   0000.318406551076,
  -0004.807497714668,
   0007.668139219592,
  -0008.933687788702,
   0008.941804096075,
  -0008.135735634262,
   0006.920792309908,
  -0005.609700337731,
   0004.370389350355,
  -0003.295044621621,
   0002.406095778821,
  -0001.701181805332,
   0001.167303977174,
  -0000.770750721265,
   0000.494748189860,
  -0000.302412270991,
   0000.179304961474,
  -0000.102472056023,
   0000.054480942822,
  -0000.029143773998,
   0000.013279311944,
  -0000.006181401661,
   0000.002276794662,
  -0000.000825870617,
   0000.000197788215,
  -0000.000043063951
};

const double b_coeffs_outer[N_POLES_OUTER+1] = {
  1.0000000000000,
  -004.0349895450688,
   011.0290231079749,
  -023.9614522030603,
   044.0276829841905,
  -071.9542585003394,
   106.4631254388799,
  -145.2718827454918,
   184.5699750348774,
  -220.1237086188597,
   247.7432282180410,
  -264.2095776342560,
   267.7259387866769,
  -258.2626159044019,
   237.4301935478215,
  -208.0750233813122,
   173.8496218145515,
  -138.2753023512930,
   104.6169424731797,
  -075.0801814106885,
   050.9842393047636,
  -032.6336122910636,
   019.5966480455533,
  -010.9732724053233,
   005.6871406568392,
  -002.6999709696586,
   001.1615008458294,
  -000.4567640976323,
   000.1858873256978,
  -000.1127367795812,
   000.1178592928037,
  -000.1300387341089,
   000.1390895340501,
  -000.1304847787657,
   000.1143236927973,
  -000.0976444282411,
   000.0843159968286,
  -000.0826145163957,
   000.0898437197281,
  -000.1060164566852,
   000.1253277628829,
  -000.1430524268088,
   000.1537980132182,
  -000.1546956468501,
   000.1443451435719,
  -000.1241126829384,
   000.0972098933477,
  -000.0674736676835,
   000.0405253784449,
  -000.0185370599851,
   000.0055939534803,
  -000.0010244823921,
   000.0039228196508,
  -000.0113597846328,
   000.0196504752834,
  -000.0254302161929,
   000.0264059957357,
  -000.0216934224724,
   000.0121238246812,
   000.0001850623860,
  -000.0125329810756,
   000.0223510664269,
  -000.0278335033989,
   000.0285932782168,
  -000.0251595428964,
   000.0190878311761,
  -000.0119740385896,
   000.0047378706186,
   000.0005474449468,
  -000.0038848977543,
   000.0055926441988,
  -000.0056811972359,
   000.0053733071706,
  -000.0046488496737,
   000.0036025559769,
  -000.0032405901805,
   000.0018759486019,
  -000.0020576866904,
   000.0007233182495,
  -000.0011645486852,
   000.0001967600266,
  -000.0005194440661,
   000.0000594759413,
  -000.0001695511137,
   000.0000276705661,
  -000.0000416866649,
   000.0000081681287,
  -000.0000075133228,
   000.0000011502916,
  -000.0000007224012
};


// ========== For Filtering ===============
volatile int g_latestBalValue;
volatile bool g_new_sample_time;
const int MIN_BAL_READING = 200;
const int MAX_BAL_READING = 700;
const double FILTER_ALPHA_MOTOR = 0.75; 
const double FILTER_ALPHA_BALL = 0.05;
double filtered_mot_raw;
double filtered_bal_raw;

// ========== For testing /input generation ==========
// --- Autostepping --- 

// --- Autostepping --- 
float pos_mag[] = {0.10, 0.25, 0.10, 0.25, 0.10, 0.25, 0.10, 0.25, 0.10};
float rad_mag[] = {0.0, -0.7, 0.7, -0.7, 0.7, -0.7, 0.7};

float* autostep_array; 
int num_steps;         

int step_index = 0;
bool autostep = true;

// =============== State setup ===============
enum ControlMode{
  STEP_INPUT = 1,
  CALIBRATE_BALL = 2,
  ISR_TIMING_TEST = 3,
  INNER_LOOP_TEST = 4   // NEW: inner loop only, theta_ref steps
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
// // station 13 NEW
const int motor_pot_min = 563;
const int motor_pot_max = 455;

// station 12
//const int motor_pot_min = 443;
//const int motor_pot_max = 336;

const double motor_pot_slope = PI / (2.0 * (motor_pot_max - motor_pot_min));
const double motor_pot_offset = PI / 4.0 - motor_pot_slope * motor_pot_max;


double ball_pos_1 = 0.1; //meters
double ball_pos_2 = 0.25; //meters
//STATION 12 VALUES
double ball_reading_1 = 412; //sensor output @ 0.1 m
double ball_reading_2 = 564; //sensor output @ 0.25 m
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
      g_target_pos = autostep_array[step_index];
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
  else if (input_mode == INNER_LOOP_TEST) {
    if (autostep) {
      // Reuse pos_mag as theta steps (in radians)
      g_target_pos = autostep_array[step_index];
      Serial.print("NEW TEST STARTED -> THETA STEP: ");
      Serial.print(g_target_pos, 4);
      Serial.println(" rad");
    } else {
      g_target_pos = userInput;
      Serial.print("NEW TEST STARTED -> MANUAL THETA: ");
      Serial.print(g_target_pos, 4);
      Serial.println(" rad");
    }
  }

  Serial.println("==============================================================================");
  if (input_mode == INNER_LOOP_TEST) {
    Serial.println("Time(ms),theta_ref(r),theta_curr(r),theta_ref_raw(r),theta_ref_sat(r),theta_curr(r),Trial,V_motor(V)");
  } else {
    Serial.println("Time(ms),y_ref(m),y_curr(m),theta_ref_raw(r),theta_ref_sat(r),theta_curr(r),Trial,V_motor(V)");
  }

  i = 0;
  outer_div_ctr = 0; // Reset outer loop counter
  currentState = RUNNING_TEST;
}




void endTest(){
  Serial.println("==============================================================================");
  // Serial.println("Test complete.");
  trial_num += 1;
  currentState = WAIT_FOR_INPUT; // Set state back to waiting

  // FIX: Check for autostep first, regardless of mode
  if (autostep) {
    step_index++; // Increment the step
    
    if (step_index < num_steps) {
      // Check which mode we're in to print the right message
      if (input_mode == STEP_INPUT || input_mode == INNER_LOOP_TEST) {
        Serial.println("Please wait for the next automatic step to occur.");
        startTest(-1); // Start the next test (userInput is ignored anyway)
      }
    } else {
      // We finished all steps
      Serial.println("\n You're finished!!! Returning to normal operation now");
      autostep = false; // Turn off autostepping
      step_index = 0; // Reset for next time
      
      // Print the correct prompt for manual input
      if (input_mode == STEP_INPUT) {
        Serial.println("\nPlease enter a new step magnitude to run another test:");
      } else if (input_mode == INNER_LOOP_TEST) {
        Serial.println("\nPlease enter a new theta step to run another test:");
      }
    }
  } 
  else {
    // Autostep is off, just print the manual prompt for the current mode
    if (input_mode == STEP_INPUT) {
      Serial.println("\nPlease enter a new step magnitude to run another test:");
    } else if (input_mode == INNER_LOOP_TEST) {
      Serial.println("\nPlease enter a new theta step to run another test:");
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
      autostep_array = pos_mag;
      num_steps = sizeof(pos_mag) / sizeof(pos_mag[0]); // Get count for pos_mag

      if(autostep){
        Serial.println("(AUTOMATED LAB 3 STEPS)");
        Serial.println("\nPress any character and Enter to start the trial:");
      }
      else{
        Serial.println("\nPlease enter a STEP POSITION (in meters) and press Enter:");
      }
      break;

    case INNER_LOOP_TEST:
      autostep_array = rad_mag; 
      num_steps = sizeof(rad_mag) / sizeof(rad_mag[0]);
      Serial.println("Mode: INNER LOOP TEST (theta_ref in radians)");
      if (autostep) {
        Serial.println("(AUTOMATED THETA STEPS)");
        Serial.println("\nPress any character and Enter to start the trial:");
      } else {
        Serial.println("\nPlease enter a THETA STEP (in radians) and press Enter:");
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
  if(g_new_sample_time){
    int newSample = analogRead(BAL_PIN);
    if (newSample > MIN_BAL_READING && newSample < MAX_BAL_READING) {
      float newAverage = (FILTER_ALPHA_BALL * (float)newSample) + ((1.0 - FILTER_ALPHA_BALL) * g_latestBalValue);
      noInterrupts();
      g_latestBalValue = newAverage;
      g_new_sample_time = false; 
      interrupts();
    }
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

  double u_new = u_total / b_coeffs_outer[0];// Assume b0 is 1, but good practice
  u_queue_outer.push(&u_new);
  
  // if (u_new > 0.01) { // Apply stiction compensation if phi angle is not tiny
  //   u_new = max(u_new, beam_stiction_offset_pos);
  // }
  // else if (u_new < -0.01) {
  //   u_new = min(u_new, beam_stiction_offset_neg);
  // }
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

  double u_new = u_total / b_coeffs_inner[0]; // Assume b0 is 1, but good practice
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
    if (input_mode == INNER_LOOP_TEST) {
      // Bypass outer loop: directly use theta reference
      g_theta_ref_from_outer_loop = g_target_pos;
    } 
    else if (outer_div_ctr == 0) {
      g_theta_ref_from_outer_loop = outer_ctrl(g_target_pos, current_y);
    }

    outer_div_ctr = (outer_div_ctr + 1) % OUTER_DIV; // increment counter and wrap around

    // ---- Inner Loop Controller (Runs at faster rate T_INNER) ----

    //Saturate the reference angle (uses the value from outer loop, which is held constant between outer loop ticks)
    double constrained_theta_ref = constrain(g_theta_ref_from_outer_loop, -saturation_limit, saturation_limit);

    // Calculate motor voltage from inner loop
    double U = inner_ctrl(constrained_theta_ref, current_angle);

    double U_comped = U;
    //   if (abs(U) > 0.01) { // Apply compensation if voltage is not tiny
    //   U_comped = U + (U > 0 ? motor_stiction_offset_pos : motor_stiction_offset_neg);
    // }

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
    g_new_sample_time = true; 
    i++;
}



