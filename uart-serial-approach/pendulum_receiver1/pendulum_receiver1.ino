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

/*// 30ms
constexpr unsigned balancePeriod = 30;

constexpr float Kxc = 1.9640327395252326;
constexpr float Kvc = 2.9835555618062073;
constexpr floatencoderInitialized Lx12 = -0.017944586125;
constexpr float Lx22 = 0.267893577575;
constexpr float Lx33 = 0.304777354501;
constexpr float Lx43 = -0.836693006901;
constexpr float Lx34 = 0.009105085454;
constexpr float Lx44 = 0.968732851241;
constexpr float Lu1 = -0.000658837496;
constexpr float Lu2 = 0.008038615579;
constexpr float Lu3 = 0.000194904229;
constexpr float Lu4 = 0.042142861797;
constexpr float Ly11 = 0.732222417486;
constexpr float Ly21 = 0.004018337431;
constexpr float Ly12 = 0.0259779136;
constexpr float Ly22 = 0.731985872302;
constexpr float Ly33 = 0.697134003054;
constexpr float Ly43 = 1.249973302544;
//*/

constexpr unsigned balancePeriod = 50; // number of milliseconds between samples. Must equal the sample period on the sensor side.

/*
q = Float64[ 1, 0.01, (max_pos/max_ang)^2, 0.01 ]
r = q[2]*2.5
v = [1.5e-4, 1.5e-4, 2.5e-3]
w = Float64[1e-3, 1e-3, 1e-1, 2.5]
*/

constexpr float Kxc = 4.76853174289176;
constexpr float Kvc = 5.494618427877538;
constexpr float Kxp = -40.617076707327456;
constexpr float Kvp = -10.763605827396049;

constexpr float Lx11 = 0.564251064459;
constexpr float Lx21 = -0.014108600623;
constexpr float Lx12 = -0.007583061891;
constexpr float Lx22 = 0.565554280146;
constexpr float Lx33 = 0.010365488563;
constexpr float Lx43 = -3.647917852394;
constexpr float Lx34 = 0.000512307372;
constexpr float Lx44 = 0.802444179543;
constexpr float Lu1 = -0.001084466925;
constexpr float Lu2 = 0.028295349758;
constexpr float Lu3 = 1.8243477e-5;
constexpr float Lu4 = 0.064185780115;
constexpr float Ly11 = 0.435748935541;
constexpr float Ly21 = 0.014108600623;
constexpr float Ly12 = 0.035795615114;
constexpr float Ly22 = 0.433740289823;
constexpr float Ly33 = 0.989813418827;
constexpr float Ly43 = 4.277365332956;

/*
constexpr float Kxc = 5.2982704480828335;
constexpr float Kvc = 5.956384314391629;
constexpr float Kxp = -42.38072141515168;
constexpr float Kvp =Received b'1418;\n'a 
at Lx21 = -0.00486277645;
constexpr float Lx12 = -0.035412629375;
constexpr float Lx22 = 0.194483714665;
constexpr float Lx33 = 0.08328663877;
constexpr float Lx43 = -2.514403619432;
constexpr float Lx34 = 0.004116386676;
constexpr float Lx44 = 0.858467365904;
constexpr float Lu1 = -0.002013476579;
constexpr float Lu2 = 0.009730264204;
constexpr float Lu3 = 0.000146586226;
constexpr flo
at Lu4 = 0.066180788871;
constexpr float Ly11 = 0.80572Lets look at a typical example:
Assu3146513;
constexpr float Ly33 = 0.918150881044;
constexpr float Ly43 = 3.163415452614;
*/

/*constexpr float Kxc = 5.334532911199483;
constexpr float Kvc = 6.138450142181159;
constexpr float Kxp = -43.133527802791534;
constexpr float Kvp = -11.527880520256758;

constexpr float Lx11 = 0.564251064459;
constexpr float Lx21 = -0.014108600623;
constexpr float Lx12 = -0.007583061891;
constexpr float Lx22 = 0.565554280146;
constexpr float Lx33 = 0.436767119852;
constexpr float Lx43 = -1.367298396208;
constexpr float Lx34 = 0.021586924134;
constexpr float Lx44 = 0.91516227791;
constexpr float Lu1 = -0.001084466925;
constexpr float Lu2 = 0.028295349758;
constexpr float Lu3 = 0.000768719266;
constexpr float Lu4 = 0.068199718055;
constexpr float Ly11 = 0.435748935541;
constexpr float Ly21 = 0.014108600623;
constexpr float Ly12 = 0.035795615114;
constexpr float Ly22 = 0.433740289823;
constexpr float Ly33 = 0.570771440933;
constexpr float Ly43 = 2.036109161267;
*/

/*// 50mslook at a 
constexpr unsigned balancePeriod = 50;

constexpr float Kxc = 0.4105195826340608;
constexpr float Kvc = 1.0524119263480645;
constexpr float Kxp = -23.03367689661796;
constexpr float Kvp = -6.133366170380374;

constexpr float Lx11 = 0.564251064459;
constexpr float Lx21 = -0.014108600623;
constexpr float Lx12 = -0.007583061891;
constexpr float Lx22 = 0.565554280146;
constexpr float Lx33 = 0.495805001322;
constexpr float Lx43 = -0.819395530424;
constexpr float Lx34 = 0.024504832123;
constexpr float Lx44 = 0.942242012301;
constexpr float Lu1 = -0.001084466925;
constexpr float Lu2 = 0.028295349758;
constexpr float Lu3 = 0.000872627172;
constexpr float Lu4 = 0.069164038557;
constexpr float Ly11 = 0.435748935541;
constexpr float Ly21 = 0.014108600623;
constexpr float Ly12 = 0.035795615114;
constexpr float Ly22 = 0.433740289823;
constexpr float Ly33 = 0.51275254793;encoderInitialized
constexpr float Ly43 = 1.497663049141;
//*/

/*// 100ms
constexpr unsigned balancePeriod = 100;

constexpr float Kxc = -1.0645508493369167;
constexpr float Kvc = -2.2074578196558834encoderInitialized;
constexpr float Kxp = 27.031267883367985;
constexpr float Kvp = 7.183781486831478;

constexpr float Lx11 = 0.0144805826;
constexpr float Lx21 = -0.0007276669;
constexpr float Lx12 = -0.0978193499;
constexpr float Lx22 = 0.0145298211;
constexpr float Lx33 = -0.1057551697;
constexpr float Lx43 = -1.8842488264;
constexpr float Lx34 = -0.0101078478;
constexpr float Lx44 = 0.7537285646;
constexpr float Lu1 = -0.0098543379;
constexpr float Lu2 = 0.0014566204;
constexpr float Lu3 = -0.0007136766;
constexpr float Lu4 = 0.1238240954;
constexpr float Ly11 = 0.9855194174;
constexpr float Ly21 = 0.0007276669;
constexpr float Ly12 = 0.0992674082;
constexpr float Ly22 = 0.9853974122;
constexpr float Ly33 = 1.0987563927;
constexpr float Ly43 = 3.098548392;
//*/


constexpr float Kxic = 0.25 * Kxc;  // integral gain (TODO: incorporate into system model)

constexpr float f_kf = 0.9;  // factor for complementary filtering of kalman estimate with "raw" measurements


// SAMPLING PERIOD

constexpr float Ts = balancePeriod / 1000.0;  // sampling period in seconds
constexpr float fs = 1000.0 / balancePeriod;  // sampling frequency in Hz

/*// bilinear filter coefficients
  constexpr float Fxc = 0.9;
  constexpr float Fvc = 0.9; constexpr float Fvc_2 = Fvc / 2;
  constexpr float Fxp = 0.9;
  constexpr float Fvp = 0.9;
//*/


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
          Serial.printf("log:%u;%f;%f;%f;%f;%f;%f;%f;%f;%f\n", k,
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