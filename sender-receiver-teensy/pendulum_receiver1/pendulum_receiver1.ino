#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <TeensyStep.h>
//#include <fastmath.h>

#include <BasicLinearAlgebra.h>
using BLA::operator<<; // Required for printing matrices to Serial

#define FeedbackSerial Serial1

//#define USE_BOUNCE2

#define NEW_BOARD

// PINS
#ifdef NEW_BOARD
constexpr unsigned pinMotorStep = 6;
constexpr unsigned pinMotorDir = 7;
constexpr unsigned pinLimitLeft = 31;
constexpr unsigned pinLimitRight = 32;
constexpr unsigned pinLedRed = 22;
constexpr unsigned pinLedGreen = 23;
#else
constexpr unsigned pinMotorStep = 8;
constexpr unsigned pinMotorDir = 9;
constexpr unsigned pinLimitLeft = 14;
constexpr unsigned pinLimitRight = 15;
constexpr unsigned pinLedRed = 23;
constexpr unsigned pinLedGreen = 22;
#endif



// LIBRARY API OBJECTS

// motor controller
Stepper motor(pinMotorStep, pinMotorDir);
RotateControl rotate(5, 1000);
StepControl driveto;

// limit switches
#ifdef USE_BOUNCE2
//#define BOUNCE_WITH_PROMPT_DETECTION
//#define BOUNCE_LOCK_OUT
#include <Bounce2.h>
Bounce limitSwitchLeft;
Bounce limitSwitchRight;
#endif


// STEPPER PARAMETERS

constexpr int motorPPR = 1600;
int motorMaxRPM = 20 * 60;  // approx. 1s for track length
int motorPPS = (motorMaxRPM * motorPPR) / 60;
int motorACC = motorPPS * 15;

int homingRPM = motorMaxRPM / 10;  // approx. 10s for track length
float homingFactor = (float)homingRPM / motorMaxRPM;


// DRIVE GEOMETRY

float REV_PER_TRACK = 20.06;
constexpr float trackLengthMeters = 1.2;
float MSTEP_PER_METER = REV_PER_TRACK * motorPPR / trackLengthMeters;
float METER_PER_MSTEP = trackLengthMeters / (REV_PER_TRACK * motorPPR);

float vMaxMeters = METER_PER_MSTEP * motorPPS;
float aMaxMeters = METER_PER_MSTEP * motorACC;

constexpr float safeRangeMeters = 1.0;
constexpr float safePosMeters = safeRangeMeters / 2;
constexpr float safePosPercentage = safePosMeters / trackLengthMeters;

int trackLengthSteps = REV_PER_TRACK * motorPPR;             // initial guess before homing
int safePosSteps = safePosPercentage * trackLengthSteps;     // initial guess
float stepsPerMeter = trackLengthSteps / trackLengthMeters;  // initial guess


// ENCODER PARAMETERS
constexpr int encoderPPRhalf = 600 * 2;
constexpr int encoderPPR = encoderPPRhalf * 2;
//constexpr int encoderOrigin = encoderPPRhalf - 2;
constexpr int encoderOrigin = encoderPPRhalf;
constexpr int angleSign = 1;
constexpr float RAD_PER_ESTEP = TWO_PI / encoderPPR;
constexpr float ESTEP_PER_RAD = encoderPPR / TWO_PI;

// Variables for remote encoder input:
int currentEncoderValue = 0;
bool encoderInitialized = false;
bool newEncoderValueAvailable = false; // set to true after every received value

bool hasPauseBeenSignaled = false;
uint32_t pauseDurationMillis;

unsigned long lastSequenceNumber = 0;

// UTILITY FUNCTIONS

int mod(int x, int y) {
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

int32_t angleSteps(int encoderPos) {
  return angleSign * (mod(encoderPos - encoderOrigin - encoderPPRhalf, encoderPPR) - encoderPPRhalf);
}

float getPoleAngleRad() {  // get encoder angle in radians
  return RAD_PER_ESTEP * angleSteps(currentEncoderValue);
}

float getCartPosMeter() {
  return METER_PER_MSTEP * motor.getPosition();
}

float getCartSpeedMeter() {  // CAUTION! RotateControl.getCurrentSpeed() only provides step frequency!
  return METER_PER_MSTEP * motor.dir * rotate.getCurrentSpeed();
}


// STATE

enum state { INIT,
             WANDER,
             HOME_LEFT,
             HOME_RIGHT,
             REACH_HOME,
             HOME,
             PERFORM_SWING_UP,
             WAIT_FOR_SWING_UP_SIGNAL,
             BALANCE,
             UNDEFINED } CartState = INIT;
int returnPosition = 0;

unsigned k = 0;

// Carabelli approach variables:
float xi_cart = 0.0;  // cart position integral [m·s]

float x_cart = 0.0;  // cart position [m]               NOT [msteps]
float v_cart = 0.0;  // cart velocity [m/s]             NOT [msteps/s]
float x_pole = 0.0;  // pole angle [rad]                NOT [esteps]
float v_pole = 0.0;  // pole angluar velocity [rad/s]   NOT [esteps/s]

//float x_cart_p = 0.0;
//float v_cart_p = 0.0;
//float x_pole_p = 0.0;
//float v_pole_p = 0.0;

float poleAngle_p = 0.0;  // previous angle for angular velocity estimation

float u_accel = 0.0;  // previous control input [m/s²]  NOT [msteps/s²]

// IST approach variables:
BLA::Matrix<4,1> x_k_k_minus_1;
BLA::Matrix<4,4> P_k_k_minus_1;
BLA::Matrix<4,3> K_k;
BLA::Matrix<4,1> x_k;
BLA::Matrix<4,4> P_k;

bool useISTApproach = false;

bool limitLeft;
bool limitRight;


// TIMING

elapsedMillis blinkTimer = 0;  // timer for LED heartbeat
constexpr unsigned blinkPeriod = 250;

elapsedMillis initTimer = 0;  // timer for initial delay
constexpr unsigned initDelay = 2000;

elapsedMillis encoderTimer = 0;  // timer for sending encoder angle over serial
constexpr unsigned encoderPeriod = 2000;

bool isUpright = false;

// elapsedMicros profileTimer = 0;  // timer for measuring execution time


// CONTROLLER AND FILTER COEFFICIENTS.

/* The remaining parameters (balancePeriod, K{x,v}{c,p}, and L{x,u,y}i[j])
 * are determined using the Julia script `$(git root)/src/demonstrator.jl>
 */

unsigned balancePeriod = 50;

 // CARABELLI APPROACH:

float Kxc = 4.371647840086138;
float Kvc = 5.153527889236189;
float Kxp = -39.38984870058722;
float Kvp = -10.434155921005745;

float Lx11 = 0.564251064459;
float Lx21 = -0.014108600623;
float Lx12 = -0.007583061891;
float Lx22 = 0.565554280146;
float Lx33 = 0.010356446081;
float Lx43 = -3.648011475727;
float Lx34 = 0.000511859164;
float Lx44 = 0.802436276753;
float Lu1 = -0.001084466925;
float Lu2 = 0.028295349758;
float Lu3 = 1.8227504e-5;
float Lu4 = 0.064185457789;
float Ly11 = 0.435748935541;
float Ly21 = 0.014108600623;
float Ly12 = 0.035795615114;
float Ly22 = 0.433740289823;
float Ly33 = 0.989822343862;
float Ly43 = 4.27759379409;


float Kxic = 0.25 * Kxc;  // integral gain (TODO: incorporate into system model)

constexpr float f_kf = 0.9;  // factor for complementary filtering of kalman estimate with "raw" measurements

// IST approach
// Constants:
BLA::Matrix<4,4> A;
BLA::Matrix<4,1> B;
BLA::Matrix<3,4> C;
BLA::Matrix<4,4> Q;
BLA::Matrix<3, 3> R; // TODO: find dimensions
BLA::Matrix<4,1> K_iqc;
float sigmaSquare;
BLA::Matrix<4,4> identityMatrix4x4 = {
  1,0,0,0,
  0,1,0,0,
  0,0,1,0,
  0,0,0,1
};

// SAMPLING PERIOD

float Ts = balancePeriod / 1000.0;  // sampling period in seconds
float fs = 1000.0 / balancePeriod;  // sampling frequency in Hz

/*// bilinear filter coefficients
  constexpr float Fxc = 0.9;
  constexpr float Fvc = 0.9; constexpr float Fvc_2 = Fvc / 2;
  constexpr float Fxp = 0.9;
  constexpr float Fvp = 0.9;
//*/

// Parameters for swing-up:
uint32_t swingUpTimeLastDirSwitch = 0;
int swingUpSampleNMinusOne, swingUpSampleNMinusTwo;
int swingUpDistance = 0.105 * trackLengthSteps; // with small sail
float swingUpSpeedFactor = 1.0;
float swingUpAccelerationFactor = 1.0;
uint32_t swingUpTimeLastPause = 0;
uint32_t timeBalancingStarted = 0;
const uint32_t balancingGracePeriodMillis = 30000;
bool sendEncoderValuesThroughFeedbackLink = false;
bool doSwingUpAtStart = false;
bool swingUpSignalReceived = false;

bool gracePeriodEnded = false;

/**
* The coefficients were calculated with the demonstrator.jl Julia script.
*/
void updateParametersForSamplingPeriod(unsigned samplingPeriod){
  if(samplingPeriod == balancePeriod){
    return; // Nothing to do, parameters already set correctly.
  }

  balancePeriod = samplingPeriod;
  Ts = balancePeriod / 1000.0;  // sampling period in seconds
  fs = 1000.0 / balancePeriod;  // sampling frequency in Hz

  switch(samplingPeriod){
    case 10:
      Kxc = 5.460879579024502;
      Kvc = 6.317330404682753;
      Kxp = -45.38283069547128;
      Kvp = -12.003680491201385;

      Lx11 = 0.77284936269;
      Lx21 = -0.003863314472;
      Lx12 = 0.00159557575;
      Lx22 = 0.773001026397;
      Lx33 = 0.486684863435;
      Lx43 = -2.298789201109;
      Lx34 = 0.004864576672;
      Lx44 = 0.976322619553;
      Lu1 = -2.2686711e-5;
      Lu2 = 0.00773020343;
      Lu3 = 3.4742919e-5;
      Lu4 = 0.014114941957;
      Ly11 = 0.22715063731;
      Ly21 = 0.003863314472;
      Ly12 = 0.006132917877;
      Ly22 = 0.226960340458;
      Ly33 = 0.513655922912;
      Ly43 = 2.437239843769;
      break;

    case 20:
      Kxc = 5.165281693403738;
      Kvc = 6.002502982442074;
      Kxp = -43.76575493919382;
      Kvp = -11.580382984925187;

      Lx11 = 0.695087028219;
      Lx21 = -0.006948599836;
      Lx12 = 0.000865478732;
      Lx22 = 0.695477928181;
      Lx33 = 0.319923009857;
      Lx43 = -2.864727541923;
      Lx34 = 0.006386532421;
      Lx44 = 0.940016271473;
      Lu1 = -0.000121707831;
      Lu2 = 0.013910948284;
      Lu3 = 9.1193586e-5;
      Lu4 = 0.02770158036;
      Ly11 = 0.304912971781;
      Ly21 = 0.006948599836;
      Ly12 = 0.013036261833;
      Ly22 = 0.304383099822;
      Ly33 = 0.680971489791;
      Ly43 = 3.136446803359;
      break;

    case 30:
      Kxc = 4.885785666306059;
      Kvc = 5.704188566930921;
      Kxp = -42.230914233541746;
      Kvp = -11.178479918391202;

      Lx11 = 0.641028155623;
      Lx21 = -0.009612707737;
      Lx12 = -0.001122240146;
      Lx22 = 0.641699049485;
      Lx33 = 0.199631190593;
      Lx43 = -3.204461975726;
      Lx34 = 0.005963885957;
      Lx44 = 0.89799554545;
      Lu1 = -0.000322129874;
      Lu2 = 0.019255297203;
      Lu3 = 0.000127663417;
      Lu4 = 0.040628643706;
      Ly11 = 0.358971844377;
      Ly21 = 0.009612707737;
      Ly12 = 0.020353084815;
      Ly22 = 0.358012569283;
      Ly33 = 0.801621034337;
      Ly43 = 3.602980216108;
      break;

    case 40:
      Kxc = 4.621517039461215;
      Kvc = 5.421481729027228;
      Kxp = -40.77371856663793;
      Kvp = -10.79677829998335;

      Lx11 = 0.598917474448;
      Lx21 = -0.011976948364;
      Lx12 = -0.004005290028;
      Lx22 = 0.599894906022;
      Lx33 = 0.099662231137;
      Lx43 = -3.449539007485;
      Lx34 = 0.003956961508;
      Lx44 = 0.851934180393;
      Lu1 = -0.000639345581;
      Lu2 = 0.0240053778;
      Lu3 = 0.000112845288;
      Lu4 = 0.05281376778;
      Ly11 = 0.401082525552;
      Ly21 = 0.011976948364;
      Ly12 = 0.027961989006;
      Ly22 = 0.399626016043;
      Ly33 = 0.901444645726;
      Ly43 = 3.967578692886;
      break;

    case 50:
      Kxc = 4.371647840086138;
      Kvc = 5.153527889236189;
      Kxp = -39.38984870058722;
      Kvp = -10.434155921005745;

      Lx11 = 0.564251064459;
      Lx21 = -0.014108600623;
      Lx12 = -0.007583061891;
      Lx22 = 0.565554280146;
      Lx33 = 0.010356446081;
      Lx43 = -3.648011475727;
      Lx34 = 0.000511859164;
      Lx44 = 0.802436276753;
      Lu1 = -0.001084466925;
      Lu2 = 0.028295349758;
      Lu3 = 1.8227504e-5;
      Lu4 = 0.064185457789;
      Ly11 = 0.435748935541;
      Ly21 = 0.014108600623;
      Ly12 = 0.035795615114;
      Ly22 = 0.433740289823;
      Ly33 = 0.989822343862;
      Ly43 = 4.27759379409;
      break;

    case 60:
      Kxc = 4.135394255924482;
      Kvc = 4.8995204988546455;
      Kxp = -38.07523998374781;
      Kvp = -10.089557077707294;

      Lx11 = 0.534743758837;
      Lx21 = -0.016051296235;
      Lx12 = -0.011725632247;
      Lx22 = 0.536387563198;
      Lx33 = -0.073103463874;
      Lx43 = -3.823334041659;
      Lx34 = -0.004313911819;
      Lx44 = 0.749677926165;
      Lu1 = -0.001666076701;
      Lu2 = 0.032212146125;
      Lu3 = -0.000184108634;
      Lu4 = 0.074672554088;
      Ly11 = 0.465256241163;
      Ly21 = 0.016051296235;
      Ly12 = 0.043810257777;
      Ly22 = 0.462649359027;
      Ly33 = 1.071297579101;
      Ly43 = 4.555782190195;
      break;

    case 70:
      Kxc = 3.912014408628015;
      Kvc = 4.658698365112583;
      Kxp = -36.82606709571432;
      Kvp = -9.761988542860353;

      Lx11 = 0.509052696704;
      Lx21 = -0.017835852496;
      Lx12 = -0.016341343592;
      Lx22 = 0.511048920824;
      Lx33 = -0.153566293631;
      Lx43 = -3.988949280747;
      Lx34 = -0.010510185776;
      Lx44 = 0.693618177368;
      Lu1 = -0.002391073158;
      Lu2 = 0.035817122296;
      Lu3 = -0.000522522933;
      Lu4 = 0.084199685531;
      Ly11 = 0.490947303296;
      Ly21 = 0.017835852496;
      Ly12 = 0.051975032361;
      Ly22 = 0.487702569501;
      Ly33 = 1.148440970687;
      Ly43 = 4.81484715618;
      break;

    case 80:
      Kxc = 3.700806223475018;
      Kvc = 4.430343114702404;
      Kxp = -35.638729677938855;
      Kvp = -9.450515774161817;

      Lx11 = 0.486312471811;
      Lx21 = -0.01948525936;
      Lx12 = -0.021361889725;
      Lx22 = 0.488670827881;
      Lx33 = -0.232931482096;
      Lx43 = -4.153471311587;
      Lx34 = -0.01809675056;
      Lx44 = 0.634087744975;
      Lu1 = -0.003265151088;
      Lu2 = 0.03915601906;
      Lu3 = -0.001026440502;
      Lu4 = 0.092684807684;
      Ly11 = 0.513687528189;
      Ly21 = 0.01948525936;
      Ly12 = 0.06026688747;
      Ly22 = 0.50977035137;
      Ly33 = 1.222863332499;
      Ly43 = 5.062598053195;
      break;

    case 90:
      Kxc = 3.5011053931851546;
      Kvc = 4.213776790103557;
      Kxp = -34.50983880441882;
      Kvp = -9.15425934713725;

      Lx11 = 0.465928605847;
      Lx21 = -0.021017337425;
      Lx12 = -0.026734372055;
      Lx22 = 0.468657226789;
      Lx33 = -0.312590492378;
      Lx43 = -4.322932100692;
      Lx34 = -0.02711495808;
      Lx44 = 0.570831536386;
      Lu1 = -0.004293104339;
      Lu2 = 0.042264270628;
      Lu3 = -0.001726802327;
      Lu4 = 0.10003760816;
      Ly11 = 0.534071394153;
      Ly21 = 0.021017337425;
      Ly12 = 0.068667946581;
      Ly22 = 0.529451212843;
      Ly33 = 1.295652633717;
      Ly43 = 5.304180991611;
      break;

    case 100:
      Kxc = 3.3122834333679854;
      Kvc = 4.008359573113829;
      Kxp = -33.43620424833006;
      Kvp = -8.872391600627838;

      Lx11 = 0.447473094237;
      Lx21 = -0.022446299066;
      Lx12 = -0.032416637155;
      Lx22 = 0.450578894971;
      Lx33 = -0.393644932636;
      Lx43 = -4.50189003442;
      Lx34 = -0.037623360302;
      Lx44 = 0.503530569853;
      Lu1 = -0.005479029187;
      Lu2 = 0.045170120992;
      Lu3 = -0.002656435505;
      Lu4 = 0.106158300495;
      Ly11 = 0.552526905763;
      Ly21 = 0.022446299066;
      Ly12 = 0.077163946578;
      Ly22 = 0.547176475122;
      Ly33 = 1.367588488055;
      Ly43 = 5.543175572319;
      break;

    case 110:
      Kxc = 3.133745826992789;
      Kvc = 3.813487630056034;
      Kxp = -32.41482250208161;
      Kvp = -8.60413348337828;

      Lx11 = 0.430626471182;
      Lx21 = -0.02378373159;
      Lx12 = -0.03837434245;
      Lx22 = 0.434115401439;
      Lx33 = -0.477025278437;
      Lx43 = -4.69403095719;
      Lx34 = -0.049695372233;
      Lx44 = 0.431813585232;
      Lu1 = -0.00682646782;
      Lu2 = 0.047896585734;
      Lu3 = -0.003850386084;
      Lu4 = 0.110936589366;
      Ly11 = 0.569373528818;
      Ly21 = 0.02378373159;
      Ly12 = 0.08574325428;
      Ly22 = 0.563268388086;
      Ly33 = 1.43925761142;
      Ly43 = 5.782185774965;
      break;

    case 120:
      Kxc = 2.964930255139581;
      Kvc = 3.628591073209003;
      Kxp = -31.442865509909684;
      Kvp = -8.348751590691506;

      Lx11 = 0.415143366674;
      Lx21 = -0.025039249947;
      Lx12 = -0.044579009442;
      Lx22 = 0.419020594209;
      Lx33 = -0.563560627555;
      Lx43 = -4.902518774473;
      Lx34 = -0.063418054411;
      Lx44 = 0.355263034612;
      Lu1 = -0.008338513373;
      Lu2 = 0.050462753905;
      Lu3 = -0.005346236689;
      Lu4 = 0.11425069762;
      Ly11 = 0.584856633326;
      Ly21 = 0.025039249947;
      Ly12 = 0.094396213443;
      Ly22 = 0.577974695797;
      Ly33 = 1.511120461125;
      Ly43 = 6.02318101729;
      break;

    case 130:
      Kxc = 2.805304911202983;
      Kvc = 3.453132033112037;
      Kxp = -30.517670073778227;
      Kvp = -8.105555380565209;

      Lx11 = 0.4008308873;
      Lx21 = -0.026220948371;
      Lx12 = -0.051006677511;
      Lx22 = 0.405100934488;
      Lx33 = -0.654022428638;
      Lx43 = -5.130212768177;
      Lx34 = -0.078891564533;
      Lx44 = 0.273417781608;
      Lu1 = -0.010017889074;
      Lu2 = 0.052884688497;
      Lu3 = -0.007184421683;
      Lu4 = 0.115966394842;
      Ly11 = 0.5991691127;
      Ly21 = 0.026220948371;
      Ly12 = 0.10311469286;
      Ly22 = 0.591490342223;
      Ly33 = 1.583551873229;
      Ly43 = 6.267703941899;
      break;

    case 140:
      Kxc = 2.65436689566438;
      Kvc = 3.2866028365176403;
      Kxp = -29.636727895037474;
      Kvp = -7.873894559184319;

      Lx11 = 0.387534439987;
      Lx21 = -0.027335723504;
      Lx12 = -0.05763694139;
      Lx22 = 0.392201288108;
      Lx33 = -0.749153544377;
      Lx43 = -5.379809707629;
      Lx34 = -0.096229039357;
      Lx44 = 0.185773745155;
      Lu1 = -0.011867009307;
      Lu2 = 0.055176070425;
      Lu3 = -0.009408547901;
      Lu4 = 0.115935991418;
      Ly11 = 0.612465560013;
      Ly21 = 0.027335723504;
      Ly12 = 0.111891762989;
      Ly22 = 0.603971710602;
      Ly33 = 1.656866979725;
      Ly43 = 6.517002660252;
      break;

    case 150:
      Kxc = 2.511640688512039;
      Kvc = 3.1285242849032437;
      Kxp = -28.7976762159642;
      Kvp = -7.653156626096831;

      Lx11 = 0.375128095634;
      Lx21 = -0.028389512031;
      Lx12 = -0.064452243753;
      Lx22 = 0.380195266084;
      Lx33 = -0.849688668499;
      Lx43 = -5.653941552753;
      Lx34 = -0.115556773687;
      Lx44 = 0.091783176954;
      Lu1 = -0.013888027639;
      Lu2 = 0.057348671923;
      Lu3 = -0.012065726555;
      Lu4 = 0.113997274676;
      Ly11 = 0.624871904366;
      Ly21 = 0.028389512031;
      Ly12 = 0.120721458098;
      Ly22 = 0.615546307112;
      Ly33 = 1.731338369865;
      Ly43 = 6.772118020597;
      break;

    case 175:
      Kxc = 2.187861456683498;
      Kvc = 2.7673928704796897;
      Kxp = -26.869024049405645;
      Kvp = -7.145394303988718;

      Lx11 = 0.347364916157;
      Lx21 = -0.030789442716;
      Lx12 = -0.082204635827;
      Lx22 = 0.353445827881;
      Lx33 = -1.12958496095;
      Lx43 = -6.464077729996;
      Lx34 = -0.173536223263;
      Lx44 = -0.174969892898;
      Lu1 = -0.019704836548;
      Lu2 = 0.062324483221;
      Lu3 = -0.02094819799;
      Lu4 = 0.099592429214;
      Ly11 = 0.652635083843;
      Ly21 = 0.030789442716;
      Ly12 = 0.142993496154;
      Ly22 = 0.641166019644;
      Ly33 = 1.924108276505;
      Ly43 = 7.440959949668;
      break;

    case 200:
      Kxc = 1.9062830095691958;
      Kvc = 2.4499150905647737;
      Kxp = -25.157202445663945;
      Kvp = -6.694239559862544;

      Lx11 = 0.323335576587;
      Lx21 = -0.032903665559;
      Lx12 = -0.100831411292;
      Lx22 = 0.330444961638;
      Lx33 = -1.460237969255;
      Lx43 = -7.487867340861;
      Lx34 = -0.247454806787;
      Lx44 = -0.49586547319;
      Lu1 = -0.02663299379;
      Lu2 = 0.066747065639;
      Lu3 = -0.033787088611;
      Lu4 = 0.06883373231;
      Ly11 = 0.676664423413;
      Ly21 = 0.032903665559;
      Ly12 = 0.165498526609;
      Ly22 = 0.66297430525;
      Ly33 = 2.128827174486;
      Ly43 = 8.163043654345;
      break;

    case 300:
      Kxc = 1.1015039040140746;
      Kvc = 1.5160406298405376;
      Kxp = -19.988214932375495;
      Kvp = -5.328641126569831;

      Lx11 = 0.251179408709;
      Lx21 = -0.039317371724;
      Lx12 = -0.181678630327;
      Lx22 = 0.262487851095;
      Lx33 = -3.606568410843;
      Lx43 = -14.858074024775;
      Lx34 = -0.779066453328;
      Lx44 = -2.621195740898;
      Lu1 = -0.06580666249;
      Lu2 = 0.080515637056;
      Lu3 = -0.151360322856;
      Lu4 = -0.31497275965;
      Ly11 = 0.748820591291;
      Ly21 = 0.039317371724;
      Ly12 = 0.25703245294;
      Ly22 = 0.725716937387;
      Ly33 = 3.121905276009;
      Ly43 = 11.768569219917;
      break;

  }
  Kxic = 0.25 * Kxc; // integral gain (TODO: incorporate into system model)
}


void pauseMotorBriefly(){
  Serial.printf("HALTING NOW for %i ms\n", pauseDurationMillis);
  //rotate.overrideSpeed(0);
  rotate.overrideAcceleration(1.0);
  rotate.stopAsync();
  uint32_t startTime = millis();
  while(millis() < startTime + pauseDurationMillis){   // wait for pauseDurationMillis
    updateRotaryEncoderValue(); // keep reading updates so that feedback to sender is not missing during pauses
  }
  //rotate.overrideSpeed(0);
  rotate.rotateAsync(motor);
  newEncoderValueAvailable = false;
}

void initializeCartState() {
  CartState = INIT;
  Serial.println("<<< INIT >>>");
  Serial.send_now();
  initTimer = 0;
}

void readInitializationValues(){
  int newMotorMaxRPM;
  float newRevolutionsPerTrack;

  // Expected Format:
  // I:motorMaxRPM;revolutionsPerTrack;doSwingUpAtStart;swingUpDistanceFactor;swingUpSpeedFactor;swingUpAccelerationFactor;\n
  // where doSwingUpAtStart in {0, 1}
  bool receivedInitValues = false;
  while(!receivedInitValues){
    Serial.println("READY");
    delay(500);

    if(Serial.available() > 0){
      if(Serial.readStringUntil(':').startsWith("I")){
        newMotorMaxRPM = Serial.readStringUntil(';').toInt();
        newRevolutionsPerTrack = Serial.readStringUntil(';').toFloat();
        doSwingUpAtStart = Serial.readStringUntil(';').toInt();
        float swingUpDistanceFactor = Serial.readStringUntil(';').toFloat();
        swingUpSpeedFactor = Serial.readStringUntil(';').toFloat();
        swingUpAccelerationFactor = Serial.readStringUntil(';').toFloat();
        receivedInitValues = true;

        swingUpDistance = swingUpDistanceFactor * trackLengthSteps;

        // Echo received values to computer for validation:
        Serial.printf("Received init values: motor max RPM: %i, revolutions per track: %f, swing up at start: %i, swing up distance factor: %f, swing up speed factor %f , swing up accel. factor: %f\n", 
        newMotorMaxRPM, newRevolutionsPerTrack, doSwingUpAtStart, swingUpDistanceFactor, swingUpSpeedFactor, swingUpAccelerationFactor);
      }
      Serial.read(); // Remove \n
    }
  }

  updateDriveGeometryParameters(newMotorMaxRPM, newRevolutionsPerTrack);
}

void updateDriveGeometryParameters(int newMotorMaxRPM, float newRevolutionsPerTrack){
  // STEPPER PARAMETERS
  motorMaxRPM = newMotorMaxRPM;  // approx. 1s for track length
  motorPPS = (motorMaxRPM * motorPPR) / 60;
  motorACC = motorPPS * 15;

  homingRPM = motorMaxRPM / 10;  // approx. 10s for track length
  homingFactor = (float)homingRPM / motorMaxRPM;


  // DRIVE GEOMETRY
  REV_PER_TRACK = newRevolutionsPerTrack;
  MSTEP_PER_METER = REV_PER_TRACK * motorPPR / trackLengthMeters;
  METER_PER_MSTEP = trackLengthMeters / (REV_PER_TRACK * motorPPR);

  vMaxMeters = METER_PER_MSTEP * motorPPS;
  aMaxMeters = METER_PER_MSTEP * motorACC;

  trackLengthSteps = REV_PER_TRACK * motorPPR;             // initial guess before homing
  safePosSteps = safePosPercentage * trackLengthSteps;     // initial guess
  stepsPerMeter = trackLengthSteps / trackLengthMeters;  // initial guess
}

void setup() {
  // set up pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinLedRed, OUTPUT);
  pinMode(pinLedGreen, OUTPUT);

#ifdef USE_BOUNCE2
  limitSwitchLeft.attach(pinLimitLeft, INPUT_PULLUP);
  limitSwitchLeft.interval(10);
  limitSwitchRight.attach(pinLimitRight, INPUT_PULLUP);
  limitSwitchRight.interval(10);
#else
  pinMode(pinLimitLeft, INPUT_PULLUP);
  pinMode(pinLimitRight, INPUT_PULLUP);
#endif


  delay(5000);
  Serial.begin(460800);
  Serial.setTimeout(20);

  readInitializationValues();

  FeedbackSerial.begin(115200);
  FeedbackSerial.setTimeout(20);

  motor.setMaxSpeed(motorPPS).setAcceleration(motorACC);
  rotate.rotateAsync(motor);
  rotate.overrideSpeed(0);
  rotate.overrideAcceleration(1.0);

  delay(500);
  Serial.println("Homing and balancing test...");

  limitLeft = digitalRead(pinLimitLeft);
  limitRight = digitalRead(pinLimitRight);

  initializeCartState();

  //CartState = REACH_HOME;
}


void checkLimitSwitches() {
#ifdef USE_BOUNCE2
  if (limitSwitchLeft.update()) {
    limitLeft = limitSwitchLeft.fell();
    digitalWriteFast(pinLedGreen, limitLeft);
    Serial.println(limitLeft ? "Left limit hit!" : "Back on track (from left)");
  }
  if (limitSwitchRight.update()) {
    limitRight = limitSwitchRight.fell();
    digitalWriteFast(pinLedRed, limitRight);
    Serial.println(limitRight ? "Right limit hit!" : "Back on track (from right)");
  }
#else
  bool limitLeftPrev = limitLeft;
  limitLeft = digitalReadFast(pinLimitLeft);
  if (limitLeft != limitLeftPrev) {
    Serial.println(limitLeft ? "Left limit hit!" : "Back on track (from left)");
    digitalWriteFast(pinLedGreen, limitLeft);
  }

  bool limitRightPrev = limitRight;
  limitRight = digitalReadFast(pinLimitRight);
  if (limitRight != limitRightPrev) {
    digitalWriteFast(pinLedRed, limitRight);
    Serial.println(limitRight ? "Right limit hit!" : "Back on track (from right)");
  }
#endif
}


void updateRotaryEncoderValue() {
  // The sample value string is of the following form:
  // encoderValue;samplingPeriodMillis;sequenceNumber;currentTime;\n
  // where the sampling period is in milliseconds
  // for example
  // S:-1204;50;1234;62345234;\n
  //
  // Additionally, pause signals of the following form can be transmitted (for pause with 800ms duration):
  // PAUSE:800;\n

  if(!(CartState == HOME || CartState == PERFORM_SWING_UP || CartState == BALANCE || CartState == WAIT_FOR_SWING_UP_SIGNAL)){
    return;
  }

  // in swing-up phase angle values come through feedback serial link
  // and in all other phases angle values come from USB serial (via network):
  Stream* serialDevice;
  if(sendEncoderValuesThroughFeedbackLink){
    serialDevice = &FeedbackSerial;

    // Regularly read from USB serial buffer to avoid packets bloating the 
    // buffer, which causes blockage at the Linux receiver:
    while(Serial.available() > 0){
      Serial.read();
    }
  } else {
    serialDevice = &Serial;
  }

    while (serialDevice->available() > 0) {
      String input1 = serialDevice->readStringUntil(':');
      if(input1.startsWith("PAUSE")){
        pauseDurationMillis = serialDevice->readStringUntil(';').toInt();
        hasPauseBeenSignaled = true;
      } else if(input1.startsWith("DOSWINGUP")){
        // Format: DOSWINGUP:1;
        serialDevice->readStringUntil(';');
        if(CartState == WAIT_FOR_SWING_UP_SIGNAL){
          swingUpSignalReceived = true;
        }
        // Also reset control parameters as this is the beginning of a new config:
        resetControlParameters();
        gracePeriodEnded = false;
        timeBalancingStarted = millis();
      } else if(input1.startsWith("DOCRASHANDSWINGUP")){
        // Format: DOCRASHANDSWINGUP:1;
        serialDevice->readStringUntil(';');
        // Also reset control parameters as this is the beginning of a new config:
        resetControlParameters();
        gracePeriodEnded = false;
        CartState = HOME;
      } else if(input1.startsWith("S")){
        // normal sampling value:
        currentEncoderValue = serialDevice->readStringUntil(';').toInt();
        unsigned delaySinceLastSample = serialDevice->readStringUntil(';').toInt();
        long sequenceNumber = serialDevice->readStringUntil(';').toInt();
        long timestamp = serialDevice->readStringUntil(';').toInt();

        updateParametersForSamplingPeriod(delaySinceLastSample);
        if(CartState == BALANCE){
          sendFeedbackToSender(sequenceNumber, timestamp);
        }
        encoderInitialized = true;
        newEncoderValueAvailable = true;
      }

      serialDevice->readStringUntil('\n');  // read newline \n
    }
  }

  void sendFeedbackToSender(unsigned long sequenceNumber, unsigned long timestamp){
    // The Feedback is of the following form:
    // sequenceNumber;Timestamp;\n
    // for example
    // FB:123;34523890;\n

    FeedbackSerial.printf("FB:%i;%i;\n", sequenceNumber, timestamp);
  }

  void sendSwingUpStartSignalToSender() {
    FeedbackSerial.print("SS;\n");
    Serial.println("Sending swing-up start signal to sender.");
  }

  void sendSwingUpEndSignalToSender() {
    FeedbackSerial.print("SE;\n");
    Serial.println("Sending swing-up end signal to sender.");
  }

  void resetControlParameters(){
    Serial.write("Resetting control parameters");

    // Carabelli approach:
    k = 0;
    xi_cart = 0.0;
    x_cart = getCartPosMeter();
    v_cart = 0.0;
    x_pole = (float)RAD_PER_ESTEP * angleSteps(currentEncoderValue);
    v_pole = 0.0;
    u_accel = 0.0;
    poleAngle_p = x_pole;

    // IST approach:
    x_k_k_minus_1.Fill(0);
    P_k_k_minus_1 = identityMatrix4x4 * sigmaSquare;
    u_accel = 0.0;
    x_k.Fill(0);
    P_k.Fill(0);

  }

  void sendGracePeriodEndedSignals(){
    delay(1); // delay to separate from other Serial messages
    String signal = "CT:GracePeriodEnded;";
    Serial.println(signal);
    Serial.send_now();
    FeedbackSerial.println(signal);
  }

  void updateUsingCarabelliKalmanAndLQR(float cartPos, float cartSpeed, float poleAngle){
    float x_cart_p = x_cart;
    float v_cart_p = v_cart;
    float x_pole_p = x_pole;
    float v_pole_p = v_pole;

    xi_cart += cartPos * Ts;

    /// KALMAN FILTER (steady-state) ///

    x_cart = Lx11 * x_cart_p + Lx12 * v_cart_p + Lu1 * u_accel + Ly11 * cartPos + Ly12 * cartSpeed;
    v_cart = Lx21 * x_cart_p + Lx22 * v_cart_p + Lu2 * u_accel + Ly21 * cartPos + Ly22 * cartSpeed;
    x_pole = Lx33 * x_pole_p + Lx34 * v_pole_p + Lu3 * u_accel + Ly33 * poleAngle;
    v_pole = Lx43 * x_pole_p + Lx44 * v_pole_p + Lu4 * u_accel + Ly43 * poleAngle;

    x_cart = f_kf * x_cart + (1 - f_kf) * cartPos;
    v_cart = f_kf * v_cart + (1 - f_kf) * cartSpeed;
    x_pole = f_kf * x_pole + (1 - f_kf) * poleAngle;
    v_pole = f_kf * v_pole + (1 - f_kf) * (poleAngle - poleAngle_p) * fs;

    poleAngle_p = poleAngle;

    //v_pole = 2*v_pole - v_pole_p; // correction

    /*
          x_cart = (float) cartPos; //Fxc * (float) cartPos + (1.0 - Fxc) * x_cart_p;
          v_cart = cartSpeed; //Fvc_2 * (x_cart - x_cart_p) * fs + Fvc_2 * cartSpeed + (1.0 - Fvc) * v_cart_p;
          x_pole = Fxp * poleAngle + (1.0 - Fxp) * x_pole_p;
          v_pole = Fvp * (x_pole - x_pole_p) * fs + (1.0 - Fvp) * v_pole_p;
  */

    //        long T2 = profileTimer;/////////////////////////////////////////////////////////////////////////////////////////

    /// CONTROLLER ///

    u_accel = Kxc * x_cart + Kvc * v_cart + Kxp * x_pole + Kvp * v_pole;  // [m/s²]
    u_accel += Kxic * xi_cart;
  }

  void updateUsingISTKalmanAndLQR(float cartPos, float cartSpeed, float poleAngle){
    BLA::Matrix<3,1> z_k = {cartPos, cartSpeed, poleAngle};

    // Kalman filter (~A means "A transposed"):
    x_k_k_minus_1 = A * x_k + B * u_accel;
    P_k_k_minus_1 = A * P_k * ~A + Q;
    K_k = P_k_k_minus_1 * ~C * BLA::Inverse(C * P_k_k_minus_1 * ~C + R);
    x_k = x_k_k_minus_1 + K_k * (z_k - C * x_k_k_minus_1);
    P_k = (identityMatrix4x4 - K_k * C) * P_k_k_minus_1;

    // LQR:
    u_accel = (~K_iqc * x_k)(0, 0);

  }

  void loop() {
    // check limit switches
    checkLimitSwitches();
    updateRotaryEncoderValue();

    // state machine
    switch (CartState) {
      case INIT:  // start wandering unless already at limit...
        if (initTimer >= initDelay) {
          if (limitLeft || limitRight) {
            Serial.println("<<< UNDEFINED >>>");
            Serial.send_now();
            CartState = UNDEFINED;
          } else {
            motor.setPosition(0);
            rotate.overrideSpeed(-homingFactor);  // expect to drive left
            Serial.println("<<< WANDER >>>");
            Serial.send_now();
            CartState = WANDER;
          }
        }
        break;

      case WANDER:  // expect to end up at the left end...
        if (limitLeft) {
          if (limitRight) {
            Serial.println("<<< UNDEFINED >>>");
            Serial.send_now();
            CartState = UNDEFINED;
            break;
          }
          returnPosition = -motor.getPosition();
          motor.setPosition(0);
          // fast return
          rotate.stop();
          motor.setTargetAbs(returnPosition);
          driveto.move(motor);
          // resume homing
          rotate.rotateAsync(motor);
          rotate.overrideSpeed(homingFactor);  // drive right
          Serial.println("<<< HOME RIGHT >>>");
          Serial.send_now();
          CartState = HOME_RIGHT;
        } else if (limitRight) {  // this should not happen in the current setup...
          rotate.stop();
          motor.setInverseRotation(true);  // unexpected direction
          rotate.rotateAsync(motor);
          rotate.overrideSpeed(-homingFactor);  // drive left
          Serial.println("<<< HOME LEFT >>>");
          Serial.send_now();
          CartState = HOME_LEFT;
        }
        break;

      case HOME_LEFT:  // wait for limitLeft (should not happen either, since WANDER will end up at left...)
        if (limitLeft) {
          returnPosition = -motor.getPosition();
          motor.setPosition(0);
          // fast return
          rotate.stop();
          motor.setTargetAbs(returnPosition);
          driveto.move(motor);
          // resume homing
          rotate.rotateAsync(motor);
          rotate.overrideSpeed(homingFactor);  // drive right
          Serial.println("<<< HOME RIGHT >>>");
          Serial.send_now();
          CartState = HOME_RIGHT;
        }
        break;

      case HOME_RIGHT:  // wait for limitRight
        if (limitRight) {
          trackLengthSteps = motor.getPosition();
          rotate.stop();
          safePosSteps = safePosPercentage * trackLengthSteps;
          stepsPerMeter = trackLengthSteps / trackLengthMeters;
          motor.setPosition(trackLengthSteps / 2);
          motor.setTargetAbs(0);
          driveto.move(motor);
          CartState = REACH_HOME;
        }
        break;

      case REACH_HOME:
        Serial.printf("Track length: %d steps = %f m\n", trackLengthSteps, trackLengthMeters);
        Serial.printf("Total revolutions: %f\n", (float)trackLengthSteps / motorPPR);
        Serial.printf("Steps per m: %f\n", stepsPerMeter);
        Serial.printf("Safe range: +/- %d steps = %f m\n", safePosSteps, safePosMeters);
        Serial.printf("Maximum speed: %d steps/s = %f m/s\n", motorPPS, vMaxMeters);
        Serial.printf("Maximum acceleration: %d steps/s² = %f m/s²\n", motorACC, aMaxMeters);
        Serial.printf("Sampling period: %f s\n", Ts);
        Serial.printf("Sampling frequency: %f Hz\n", fs);
        Serial.println("<<< HOME >>>");
        Serial.send_now();
        CartState = HOME;
        break;
      case HOME:  // at home
        if(!doSwingUpAtStart){
          if (encoderInitialized && encoderTimer >= encoderPeriod) {
            encoderTimer -= encoderPeriod;
            int encoderAbs = currentEncoderValue;
            int encoderRel = angleSteps(encoderAbs);
            bool wasUpright = isUpright;
            isUpright = (abs(encoderRel) < 20);
            if (isUpright && wasUpright) {
              isUpright = false;
              resetControlParameters();
              rotate.rotateAsync(motor);
              rotate.overrideAcceleration(0);
              rotate.overrideSpeed(0);
              Serial.println("<<< BALANCE >>>");
              Serial.send_now();
              CartState = BALANCE;
              timeBalancingStarted = millis();
            } else
              Serial.printf("Encoder_Test: %d -> angle = %d steps = %f°\n", encoderAbs, encoderRel, RAD_TO_DEG * RAD_PER_ESTEP * encoderRel);
            Serial.send_now();
          }
        } else {
          delay(4000);
          resetControlParameters();

          motor.setMaxSpeed((double)motorPPS * swingUpSpeedFactor).setAcceleration((double) motorACC * swingUpAccelerationFactor);

          sendEncoderValuesThroughFeedbackLink = true;
          sendSwingUpStartSignalToSender();
          CartState = PERFORM_SWING_UP;
        }
        break;

      case PERFORM_SWING_UP:
        // Cancel if limit switch gets hit:
        if (limitLeft || limitRight) {
          rotate.overrideAcceleration(1);
          rotate.stop();
          if (limitLeft && limitRight) {
            Serial.println("<<< UNDEFINED >>>");
            CartState = UNDEFINED;
          } else {
            motor.setTargetRel((limitLeft ? 1 : -1) * (int)motorPPR);
            driveto.move(motor);
            initializeCartState();
          }
          break;
        }

        // Swing up:
        if (newEncoderValueAvailable) {
          newEncoderValueAvailable = false;
          ++k;

          int swingUpSampleN = currentEncoderValue;

          // Check if pendulum is upright and then go into balancing mode:
          int upRightThreshold = 40;
          int maxChangeLastTwoSamples = 15;
          if (abs(angleSteps(swingUpSampleN)) < upRightThreshold 
            && abs(angleSteps(swingUpSampleNMinusOne)) < upRightThreshold 
            && abs(angleSteps(swingUpSampleNMinusTwo)) < upRightThreshold 
            && abs(swingUpSampleN - swingUpSampleNMinusOne) + abs(swingUpSampleNMinusOne - swingUpSampleNMinusTwo) < maxChangeLastTwoSamples) {
            Serial.println("Going to balance mode");

            // Restore motor maximum values:
            motor.setMaxSpeed(motorPPS).setAcceleration(motorACC);

            // Tell sender to now send angle values via network and not via feedback link:
            // sendSwingUpEndSignalToSender();

            // Initialize parameters and motor for balancing:
            resetControlParameters();
            rotate.rotateAsync(motor);
            rotate.overrideAcceleration(0);
            rotate.overrideSpeed(0);
            Serial.println("<<< BALANCE >>>");
            Serial.send_now();
            timeBalancingStarted = millis();
            CartState = BALANCE;
            break;
          }

          bool isTurningPoint = (swingUpSampleNMinusOne - swingUpSampleNMinusTwo) * (swingUpSampleN - swingUpSampleNMinusOne) <= 0;
          if ((millis() - swingUpTimeLastDirSwitch > 300 && isTurningPoint)) {

            swingUpDistance = -swingUpDistance;  // Change direction of cart

            motor.setTargetAbs(swingUpDistance);
            driveto.move(motor);

            swingUpTimeLastDirSwitch = millis();
          }
          swingUpSampleNMinusTwo = swingUpSampleNMinusOne;
          swingUpSampleNMinusOne = swingUpSampleN;
        }

        // If not successful after certain time, give a pause to restore sync between motor and pendulum movement:
        if (millis() - swingUpTimeLastPause > 15000) {
          Serial.println("Swing-up Pause");
          rotate.overrideAcceleration(0);
          rotate.overrideSpeed(0);
          delay(2000);
          swingUpTimeLastPause = millis();
        }
        break;

      case BALANCE:  // balancing loop
        if (limitLeft || limitRight) {
          rotate.overrideAcceleration(1);
          rotate.stop();
          if (limitLeft && limitRight) {
            Serial.println("<<< UNDEFINED >>>");
            CartState = UNDEFINED;
          } else {
            motor.setTargetRel((limitLeft ? 1 : -1) * (int)motorPPR);
            driveto.move(motor);
            initializeCartState();
          }
          break;
        }

        if(hasPauseBeenSignaled){
          hasPauseBeenSignaled = false;
          //pauseMotorBriefly();
        }

        if (newEncoderValueAvailable) {
          newEncoderValueAvailable = false;
          ++k;

          // u_accel = 0.0;  // control disabled

          //        profileTimer = 0;///////////////////////////////////////////////////////////////////////////////////////////////

          int32_t cartSteps = motor.getPosition();

          if (abs(cartSteps) >= safePosSteps) {
            rotate.overrideAcceleration(1);
            rotate.stop();
            if(millis() - timeBalancingStarted <= balancingGracePeriodMillis){
              delay(2000);
              motor.setTargetAbs(0);
              driveto.move(motor);
              Serial.write("Balancing failed within beginning period. Swinging up again.");
              delay(4000); // Wait for pendulum to loose some energy (overshoots otherwise).
              CartState = HOME;
            } else{
              CartState = WAIT_FOR_SWING_UP_SIGNAL;
              Serial.println("Crashed. Waiting for swing-up signal.");
            }
            break;
          }

          if(!gracePeriodEnded && millis() - timeBalancingStarted > balancingGracePeriodMillis){
            if(sendEncoderValuesThroughFeedbackLink){
              sendEncoderValuesThroughFeedbackLink = false;
              sendSwingUpEndSignalToSender();
            }
            gracePeriodEnded = true;
            sendGracePeriodEndedSignals();
          }

          float cartPos = METER_PER_MSTEP * cartSteps;
          float cartSpeed = getCartSpeedMeter();
          float poleAngle = getPoleAngleRad();

          //        long T1 = profileTimer;////////////LED_BUILTIN/////////////////////////////////////////////////////////////////////////////

        //   float x_cart_p = x_cart;
        //   float v_cart_p = v_cart;
        //   float x_pole_p = x_pole;
        //   float v_pole_p = v_pole;

        //   xi_cart += cartPos * Ts;

        //   /// KALMAN FILTER (steady-state) ///

        //   x_cart = Lx11 * x_cart_p + Lx12 * v_cart_p + Lu1 * u_accel + Ly11 * cartPos + Ly12 * cartSpeed;
        //   v_cart = Lx21 * x_cart_p + Lx22 * v_cart_p + Lu2 * u_accel + Ly21 * cartPos + Ly22 * cartSpeed;
        //   x_pole = Lx33 * x_pole_p + Lx34 * v_pole_p + Lu3 * u_accel + Ly33 * poleAngle;
        //   v_pole = Lx43 * x_pole_p + Lx44 * v_pole_p + Lu4 * u_accel + Ly43 * poleAngle;

        //   x_cart = f_kf * x_cart + (1 - f_kf) * cartPos;
        //   v_cart = f_kf * v_cart + (1 - f_kf) * cartSpeed;
        //   x_pole = f_kf * x_pole + (1 - f_kf) * poleAngle;
        //   v_pole = f_kf * v_pole + (1 - f_kf) * (poleAngle - poleAngle_p) * fs;

        //   poleAngle_p = poleAngle;

        //   //v_pole = 2*v_pole - v_pole_p; // correction

        //   /*
        //         x_cart = (float) cartPos; //Fxc * (float) cartPos + (1.0 - Fxc) * x_cart_p;
        //         v_cart = cartSpeed; //Fvc_2 * (x_cart - x_cart_p) * fs + Fvc_2 * cartSpeed + (1.0 - Fvc) * v_cart_p;
        //         x_pole = Fxp * poleAngle + (1.0 - Fxp) * x_pole_p;
        //         v_pole = Fvp * (x_pole - x_pole_p) * fs + (1.0 - Fvp) * v_pole_p;
        // */

        //   //        long T2 = profileTimer;/////////////////////////////////////////////////////////////////////////////////////////

        //   /// CONTROLLER ///

        //   u_accel = Kxc * x_cart + Kvc * v_cart + Kxp * x_pole + Kvp * v_pole;  // [m/s²]
        //   u_accel += Kxic * xi_cart;

        if(useISTApproach){
          updateUsingISTKalmanAndLQR(cartPos, cartSpeed, poleAngle);
        } else {
          updateUsingCarabelliKalmanAndLQR(cartPos, cartSpeed, poleAngle);
        }


          u_accel = constrain(u_accel, -aMaxMeters, aMaxMeters);  // clamp to admissible range

          float target_speed = cartSpeed + u_accel * Ts;                    // [m/s]
          target_speed = constrain(target_speed, -vMaxMeters, vMaxMeters);  // clamp to admissible range

          u_accel = (target_speed - cartSpeed) * fs;  // correct acceleration for speed clamping

          float accel_factor = abs(u_accel) / aMaxMeters;  // (factor must be positive, target_speed determines sign)
          float speed_factor = target_speed / vMaxMeters;

          //        long T3 = profileTimer;/////////////////////////////////////////////////////////////////////////////////////////

          rotate.overrideAcceleration(accel_factor);
          rotate.overrideSpeed(speed_factor);

          //        long T4 = profileTimer;/////////////////////////////////////////////////////////////////////////////////////////
          //        Serial.printf("%d + %d + %d + %d = %d\n", T1, T2-T1, T3-T2, T4-T3, T4);

        //if (k % 10 == 0) {
        if(!sendEncoderValuesThroughFeedbackLink){
          Serial.printf("log:%u;%f;%f;%f;%f;%f;%f;%f;%f;%f\n", k,
                        cartPos, cartSpeed, poleAngle,
                        x_cart, v_cart, x_pole, v_pole,
                        u_accel, target_speed);
          Serial.send_now();
        }
        //}
      }
      break;

      case WAIT_FOR_SWING_UP_SIGNAL:
        // This state is entered after a crash. The controller can send a swing-up signal via serial to make
        // the pendulum swing up again.
        if(swingUpSignalReceived){
          Serial.println("Do-Swing-up signal received.");
          CartState = HOME;
          swingUpSignalReceived = false;
        }
      
      case UNDEFINED:  // don't know which direction to take
        break;

      default:
        CartState = UNDEFINED;
    }

    // LED heartbeat
    if (blinkTimer >= blinkPeriod) {
      blinkTimer -= blinkPeriod;
      digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));  // toggle built-in LED
      if (CartState == UNDEFINED) {                                  // toggle warning LEDs
        digitalWriteFast(pinLedGreen, !digitalReadFast(pinLedGreen));
        digitalWriteFast(pinLedRed, !digitalReadFast(pinLedRed));
      }
    }
  }