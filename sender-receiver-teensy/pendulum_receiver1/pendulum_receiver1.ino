#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <TeensyStep.h>
//#include <fastmath.h>
#include "CarabelliParameters.h"

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
float currentAngularVelocity = 0;
int currentLatencyMillis = 0;
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

float getAngularVelocityRadPerSecond(){
  return currentAngularVelocity;
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


// IST approach Constants:
const BLA::Matrix<4,4> identityMatrix4x4 = {
  1,0,0,0,
  0,1,0,0,
  0,0,1,0,
  0,0,0,1
};
const BLA::Matrix<4,4> A_cont = {
  0,    1.0000,         0,         0,
  0,         0,         0,         0,
  0,         0,         0,    1.0000,
  0,         0,   14.0126,         0
};
const BLA::Matrix<4,1> B_cont = {0, 1.0000, 0, 1.4286};
const BLA::Matrix<4,4> C = identityMatrix4x4;

float q0_factor = 1e-6;
float r_factor = 1e-6;
BLA::Matrix<4,4> Q_0;
BLA::Matrix<4, 4> R;
BLA::Matrix<4,1> K_iqc = {13.6723,   13.5022,  -74.6153,  -19.8637};
float sigmaSquare = 1; // TODO: Find value experimentally (dummy value)

float K_iqc_integrator = 5.0322;


// IST approach variables:
BLA::Matrix<4,4> A;
BLA::Matrix<4,1> B;
BLA::Matrix<4,4> Q;

BLA::Matrix<4,1> x_k_k_minus_1;
BLA::Matrix<4,4> P_k_k_minus_1;
BLA::Matrix<4,4> K_k;
BLA::Matrix<4,1> x_k;
BLA::Matrix<4,4> P_k;


enum approach {
  IST_KALMAN_IST_CONTROLLER,
  IST_KALMAN_CARABELLI_CONTROLLER,
  CARABELLI_KALMAN_CARABELLI_CONTROLLER,
  CARABELLI_KALMAN_IST_CONTROLLER
} approachUsed = IST_KALMAN_IST_CONTROLLER;

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
float Kxic = 0.25 * Kxc;  // integral gain (TODO: incorporate into system model)

constexpr float f_kf = 0.9;  // factor for complementary filtering of kalman estimate with "raw" measurements


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

  if(approachUsed == IST_KALMAN_IST_CONTROLLER || approachUsed == IST_KALMAN_CARABELLI_CONTROLLER){
    A = identityMatrix4x4 + A_cont * Ts;
    B = B_cont * Ts;
    Q = Q_0 * Ts;
  }
  if(approachUsed == CARABELLI_KALMAN_CARABELLI_CONTROLLER || approachUsed == IST_KALMAN_CARABELLI_CONTROLLER 
    || approachUsed == CARABELLI_KALMAN_IST_CONTROLLER) {
    updateCarabelliParameters(samplingPeriod);
    Kxic = 0.25 * Kxc; // integral gain (TODO: incorporate into system model)
  }
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

void setApproachFromParameter(int parameter){
  switch(parameter){
    case 0: approachUsed = IST_KALMAN_IST_CONTROLLER; break;
    case 1: approachUsed = CARABELLI_KALMAN_CARABELLI_CONTROLLER; break;
    case 2: approachUsed = IST_KALMAN_CARABELLI_CONTROLLER; break;
    case 3: approachUsed = CARABELLI_KALMAN_IST_CONTROLLER; break;
  }
}

String getUsedApproachString(){
  switch(approachUsed){
    case IST_KALMAN_IST_CONTROLLER: return "IST_KALMAN_IST_CONTROLLER";
    case CARABELLI_KALMAN_CARABELLI_CONTROLLER: return "CARABELLI_KALMAN_CARABELLI_CONTROLLER";
    case IST_KALMAN_CARABELLI_CONTROLLER: return "IST_KALMAN_CARABELLI_CONTROLLER";
    case CARABELLI_KALMAN_IST_CONTROLLER: return "CARABELLI_KALMAN_IST_CONTROLLER";
  }
  return "";
}

void readInitializationValues(){
  int newMotorMaxRPM;
  float newRevolutionsPerTrack;

  // Expected Format:
  // I:motorMaxRPM;revolutionsPerTrack;doSwingUpAtStart;swingUpDistanceFactor;swingUpSpeedFactor;swingUpAccelerationFactor;approach\n
  // where doSwingUpAtStart in {0, 1}
  // and approach in {0, 1, 2} with 0: IST_KALMAN_IST_CONTROLLER, 1: CARABELLI_KALMAN_CARABELLI_CONTROLLER, 2: IST_KALMAN_CARABELLI_CONTROLLER
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

        K_iqc(0) = Serial.readStringUntil(';').toFloat();
        K_iqc(1) = Serial.readStringUntil(';').toFloat();
        K_iqc(2) = Serial.readStringUntil(';').toFloat();
        K_iqc(3) = Serial.readStringUntil(';').toFloat();
        K_iqc_integrator = Serial.readStringUntil(';').toFloat();
        r_factor = Serial.readStringUntil(';').toFloat();
        q0_factor = Serial.readStringUntil(';').toFloat();
        sigmaSquare = Serial.readStringUntil(';').toFloat();
        setApproachFromParameter(Serial.readStringUntil(';').toInt());

        R = identityMatrix4x4 * r_factor;
        Q_0 = identityMatrix4x4 * q0_factor;

        receivedInitValues = true;

        swingUpDistance = swingUpDistanceFactor * trackLengthSteps;

        // Echo received values to computer for validation:
        Serial.printf("Received init values: motor max RPM: %i, revolutions per track: %f, swing up at start: %i, "
          "swing up distance factor: %f, swing up speed factor %f , swing up accel. factor: %f, "
          "K_iqc: [%f, %f, %f, %f], K_iqc_integrator: %f, r_factor: %f, q0_factor: %f, sigmaSquare: %f, approach: %s\n", 
        newMotorMaxRPM, newRevolutionsPerTrack, doSwingUpAtStart, swingUpDistanceFactor, swingUpSpeedFactor, swingUpAccelerationFactor,
        K_iqc(0), K_iqc(1), K_iqc(2), K_iqc(3), K_iqc_integrator, r_factor, q0_factor, sigmaSquare, getUsedApproachString().c_str());
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
  // encoderValue;samplingPeriodMillis;sequenceNumber;currentTime;angularVelocity;networkDelayMillis\n
  // where the sampling period is in milliseconds and the angular velocity is in radians per second.
  // for example
  // S:-1204;50;1234;62345234;-1.308997;4\n
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
        currentAngularVelocity = serialDevice->readStringUntil(';').toFloat();
        currentLatencyMillis = serialDevice->readStringUntil(';').toInt();

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
    A = identityMatrix4x4 + A_cont * Ts;
    B = B_cont * Ts;
    Q = Q_0 * Ts;
    xi_cart = 0.0;

  }

  void sendGracePeriodEndedSignals(){
    delay(1); // delay to separate from other Serial messages
    String signal = "CT:GracePeriodEnded;";
    Serial.println(signal);
    Serial.send_now();
    FeedbackSerial.println(signal);
  }

  void updateUsingCarabelliKalmanAndLQR(float cartPos, float cartSpeed, float poleAngle, unsigned latency){
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


    /// Smith Predictor ///

    // Apply Smith predictor to compensate for network delay on pole angle data:
    updateSmithPredictorParameters(latency);
    BLA::Matrix<4,1> x_k = {x_cart, v_cart, x_pole, v_pole};
    x_k = LQRMatrixA * x_k + LQRMatrixB * u_accel;
    // Note: We do NOT update x_cart and v_cart from x_k, since these values are retrieved locally and don't 
    // have a network delay, i.e., no prediction necessary!
    x_pole = x_k(2);
    v_pole = x_k(3);


    /// CONTROLLER ///

    u_accel = Kxc * x_cart + Kvc * v_cart + Kxp * x_pole + Kvp * v_pole;  // [m/s²]
    u_accel += Kxic * xi_cart;
  }

  void updateUsingISTKalmanAndLQR(float cartPos, float cartSpeed, float poleAngle, float angularVelocity){
    BLA::Matrix<4,1> z_k = {cartPos, cartSpeed, poleAngle, angularVelocity};

    // Kalman filter (~A means "A transposed"):
    x_k_k_minus_1 = A * x_k + B * u_accel;
    P_k_k_minus_1 = A * P_k * ~A + Q;
    K_k = P_k_k_minus_1 * ~C * BLA::Inverse(C * P_k_k_minus_1 * ~C + R);
    x_k = x_k_k_minus_1 + K_k * (z_k - C * x_k_k_minus_1);
    P_k = (identityMatrix4x4 - K_k * C) * P_k_k_minus_1;

    // LQR:
    u_accel = (~K_iqc * x_k)(0, 0);
    xi_cart += cartPos * Ts; // integrator
    u_accel += K_iqc_integrator * xi_cart;
  }

  void updateKalmanUsingISTUpdateLQRUsingCarabelli(float cartPos, float cartSpeed, float poleAngle, float angularVelocity){
    BLA::Matrix<4,1> z_k = {cartPos, cartSpeed, poleAngle, angularVelocity};

    // Kalman filter (~A means "A transposed"):
    x_k_k_minus_1 = A * x_k + B * u_accel;
    P_k_k_minus_1 = A * P_k * ~A + Q;
    K_k = P_k_k_minus_1 * ~C * BLA::Inverse(C * P_k_k_minus_1 * ~C + R);
    x_k = x_k_k_minus_1 + K_k * (z_k - C * x_k_k_minus_1);
    P_k = (identityMatrix4x4 - K_k * C) * P_k_k_minus_1;

    u_accel = Kxc * x_k(0) + Kvc * x_k(1) + Kxp * x_k(2) + Kvp * x_k(3);  // [m/s²]
    u_accel += Kxic * xi_cart;
  }

  void updateKalmanUsingCarabelliUpdateLQRUsingIST(float cartPos, float cartSpeed, float poleAngle){
    // Kalman filter from Carabelli
    float x_cart_p = x_cart;
    float v_cart_p = v_cart;
    float x_pole_p = x_pole;
    float v_pole_p = v_pole;

    x_cart = Lx11 * x_cart_p + Lx12 * v_cart_p + Lu1 * u_accel + Ly11 * cartPos + Ly12 * cartSpeed;
    v_cart = Lx21 * x_cart_p + Lx22 * v_cart_p + Lu2 * u_accel + Ly21 * cartPos + Ly22 * cartSpeed;
    x_pole = Lx33 * x_pole_p + Lx34 * v_pole_p + Lu3 * u_accel + Ly33 * poleAngle;
    v_pole = Lx43 * x_pole_p + Lx44 * v_pole_p + Lu4 * u_accel + Ly43 * poleAngle;

    x_cart = f_kf * x_cart + (1 - f_kf) * cartPos;
    v_cart = f_kf * v_cart + (1 - f_kf) * cartSpeed;
    x_pole = f_kf * x_pole + (1 - f_kf) * poleAngle;
    v_pole = f_kf * v_pole + (1 - f_kf) * (poleAngle - poleAngle_p) * fs;
    BLA::Matrix<4,1> x_k_carabelli = {x_cart, v_cart, x_pole, v_pole};

    poleAngle_p = poleAngle;

    

    // Controller from IST:
    u_accel = (~K_iqc * x_k_carabelli)(0, 0);
    xi_cart += cartPos * Ts; // integrator
    u_accel += K_iqc_integrator * xi_cart;
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
          float angularVelocity = getAngularVelocityRadPerSecond();

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

        if(approachUsed == IST_KALMAN_IST_CONTROLLER){
          updateUsingISTKalmanAndLQR(cartPos, cartSpeed, poleAngle, angularVelocity);
        } else if (approachUsed == CARABELLI_KALMAN_CARABELLI_CONTROLLER){
          updateUsingCarabelliKalmanAndLQR(cartPos, cartSpeed, poleAngle, currentLatencyMillis);
        } else if (approachUsed == IST_KALMAN_CARABELLI_CONTROLLER){
          updateKalmanUsingISTUpdateLQRUsingCarabelli(cartPos, cartSpeed, poleAngle, angularVelocity);
        }  else if (approachUsed == CARABELLI_KALMAN_IST_CONTROLLER){
          updateKalmanUsingCarabelliUpdateLQRUsingIST(cartPos, cartSpeed, poleAngle);
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
          if(approachUsed == IST_KALMAN_IST_CONTROLLER || approachUsed == IST_KALMAN_CARABELLI_CONTROLLER){
            Serial.printf("log:%u;%f;%f;%f;%f;%f;%f;%f;%f;%f\n", k,
                          cartPos, cartSpeed, poleAngle,
                          x_k(0), x_k(1), x_k(2), x_k(3),
                          u_accel, target_speed);
          } else {
            Serial.printf("log:%u;%f;%f;%f;%f;%f;%f;%f;%f;%f\n", k,
                          cartPos, cartSpeed, poleAngle,
                          x_cart, v_cart, x_pole, v_pole,
                          u_accel, target_speed);
          }
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