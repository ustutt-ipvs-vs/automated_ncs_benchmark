#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <TeensyStep.h>
//#include <fastmath.h>

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
constexpr int motorMaxRPM = 20 * 60;  // approx. 1s for track length
constexpr int motorPPS = (motorMaxRPM * motorPPR) / 60;
constexpr int motorACC = motorPPS * 15;

constexpr int homingRPM = 120;  // approx. 10s for track length
constexpr float homingFactor = (float)homingRPM / motorMaxRPM;


// DRIVE GEOMETRY

constexpr float REV_PER_TRACK = 20.06;
constexpr float trackLengthMeters = 1.2;
constexpr float MSTEP_PER_METER = REV_PER_TRACK * motorPPR / trackLengthMeters;
constexpr float METER_PER_MSTEP = trackLengthMeters / (REV_PER_TRACK * motorPPR);

constexpr float vMaxMeters = METER_PER_MSTEP * motorPPS;
constexpr float aMaxMeters = METER_PER_MSTEP * motorACC;

constexpr float safeRangeMeters = 1.0;
constexpr float safePosMeters = safeRangeMeters / 2;
constexpr float safePosPercentage = safePosMeters / trackLengthMeters;

int trackLengthSteps = REV_PER_TRACK * motorPPR;             // initial guess before homing
int safePosSteps = safePosPercentage * trackLengthSteps;     // initial guess
float stepsPerMeter = trackLengthSteps / trackLengthMeters;  // initial guess


// ENCODER PARAMETERS
constexpr int encoderPPRhalf = 600 * 2;
constexpr int encoderPPR = encoderPPRhalf * 2;
constexpr int encoderOrigin = encoderPPRhalf - 2;
constexpr int angleSign = 1;
constexpr float RAD_PER_ESTEP = TWO_PI / encoderPPR;
constexpr float ESTEP_PER_RAD = encoderPPR / TWO_PI;

// Variables for remote encoder input:
int currentEncoderValue = 0;
bool encoderInitialized = false;
bool newEncoderValueAvailable = false; // set to true after every received value

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
             BALANCE,
             UNDEFINED } CartState = INIT;
int returnPosition = 0;

unsigned k = 0;

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


// CONTROLLER AND FILTER COEFFICIENTS

/* The remaining parameters (balancePeriod, K{x,v}{c,p}, and L{x,u,y}i[j])
 * are determined using the Julia script `$(git root)/src/demonstrator.jl>
 */

unsigned balancePeriod = 50;

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


// SAMPLING PERIOD

float Ts = balancePeriod / 1000.0;  // sampling period in seconds
float fs = 1000.0 / balancePeriod;  // sampling frequency in Hz

/*// bilinear filter coefficients
  constexpr float Fxc = 0.9;
  constexpr float Fvc = 0.9; constexpr float Fvc_2 = Fvc / 2;
  constexpr float Fxp = 0.9;
  constexpr float Fvp = 0.9;
//*/


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
  }
  Kxic = 0.25 * Kxc; // integral gain (TODO: incorporate into system model)
}

void initializeCartState() {
  CartState = INIT;
  Serial.println("<<< INIT >>>");
  Serial.send_now();
  initTimer = 0;
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

  motor.setMaxSpeed(motorPPS).setAcceleration(motorACC);
  rotate.rotateAsync(motor);
  rotate.overrideSpeed(0);
  rotate.overrideAcceleration(1.0);

  // Serial.begin(12000000);
  Serial.begin(460800);
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
  while (Serial.available() > 0) {
    currentEncoderValue = Serial.readStringUntil(';').toInt();
    unsigned delaySinceLastSample = Serial.readStringUntil(';').toInt();
    updateParametersForSamplingPeriod(delaySinceLastSample);

    encoderInitialized = true;
    Serial.read();  // read newline
    //Serial.printf("Encoder Value: %i\n", currentEncoderValue);
    newEncoderValueAvailable = true;
  }
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
      if (encoderInitialized && encoderTimer >= encoderPeriod) {
        encoderTimer -= encoderPeriod;
        int encoderAbs = currentEncoderValue;
        int encoderRel = angleSteps(encoderAbs);
        bool wasUpright = isUpright;
        isUpright = (std::abs(encoderRel) < 20);
        if (isUpright && wasUpright) {
          isUpright = false;
          k = 0;
          xi_cart = 0.0;
          x_cart = getCartPosMeter();
          v_cart = 0.0;
          x_pole = (float)RAD_PER_ESTEP * encoderRel;
          v_pole = 0.0;
          u_accel = 0.0;
          poleAngle_p = x_pole;
          rotate.rotateAsync(motor);
          rotate.overrideAcceleration(0);
          rotate.overrideSpeed(0);
          Serial.println("<<< BALANCE >>>");
          Serial.send_now();
          CartState = BALANCE;
        } else
          Serial.printf("Encoder: %d -> angle = %d steps = %f°\n", encoderAbs, encoderRel, RAD_TO_DEG * RAD_PER_ESTEP * encoderRel);
        Serial.send_now();
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
      if (newEncoderValueAvailable) {
        newEncoderValueAvailable = false;
        ++k;

        // u_accel = 0.0;  // control disabled

        //        profileTimer = 0;///////////////////////////////////////////////////////////////////////////////////////////////

        int32_t cartSteps = motor.getPosition();

        if (std::abs(cartSteps) >= safePosSteps) {
          rotate.overrideAcceleration(1);
          rotate.stop();
          initializeCartState();
          break;
        }

        float cartPos = METER_PER_MSTEP * cartSteps;
        float cartSpeed = getCartSpeedMeter();
        float poleAngle = getPoleAngleRad();

        //        long T1 = profileTimer;////////////LED_BUILTIN/////////////////////////////////////////////////////////////////////////////

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
        u_accel = constrain(u_accel, -aMaxMeters, aMaxMeters);  // clamp to admissible range

        float target_speed = cartSpeed + u_accel * Ts;                    // [m/s]
        target_speed = constrain(target_speed, -vMaxMeters, vMaxMeters);  // clamp to admissible range

        u_accel = (target_speed - cartSpeed) * fs;  // correct acceleration for speed clamping

        float accel_factor = std::abs(u_accel) / aMaxMeters;  // (factor must be positive, target_speed determines sign)
        float speed_factor = target_speed / vMaxMeters;

        //        long T3 = profileTimer;/////////////////////////////////////////////////////////////////////////////////////////

        rotate.overrideAcceleration(accel_factor);
        rotate.overrideSpeed(speed_factor);

        //        long T4 = profileTimer;/////////////////////////////////////////////////////////////////////////////////////////
        //        Serial.printf("%d + %d + %d + %d = %d\n", T1, T2-T1, T3-T2, T4-T3, T4);

        if (k % 10 == 0) {
          Serial.printf("%5u [% 1.3f % 1.3f % 1.3f * ] -> [% 1.3f % 1.3f % 1.3f % 1.3f] a% 1.3f v% 1.3f\n", k,
                        cartPos, cartSpeed, poleAngle,
                        x_cart, v_cart, x_pole, v_pole,
                        u_accel, target_speed);
          Serial.send_now();
        }
      }
      break;

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