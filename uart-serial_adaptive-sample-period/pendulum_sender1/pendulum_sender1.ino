#include <Encoder.h>

constexpr unsigned pinEncoderA = 2;
constexpr unsigned pinEncoderB = 3;
int samplingPeriodMillis = 50;
constexpr int encoderNumSteps = 2400;

int previousEncoderValue = 0;

float compareValue = 0;


Encoder encoder(pinEncoderA, pinEncoderB);

void setup() {
  Serial.begin(460800);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  int encoderValue = encoder.read();  
  Serial.printf("%i;%i;\n", encoderValue, samplingPeriodMillis);

  // Must be updated AFTER transmission, as the value just sent still has the
  // old sampling period:
  samplingPeriodMillis = angleToSamplePeriod(encoderValue);
  previousEncoderValue = encoderValue;
  delay(samplingPeriodMillis);

}


int angleToSamplePeriod(int encoderValue){
  int changeFromPrevious = abs(previousEncoderValue - encoderValue);

  float alpha = 0.5 * ((float) samplingPeriodMillis / 100.0);
  compareValue = alpha * (float) changeFromPrevious + (1-alpha) * compareValue;

  if(compareValue <= 0.5){
    return 100;
  } else if(compareValue <= 1.0){
    return 50;
  } else if(compareValue <= 3.0){
    return 20;
  } 
  return 10;
}

int angleToSamplePeriod1(int encoderValue){
  int derivFromUpwards = abs(encoderValue - encoderNumSteps/2) % encoderNumSteps; // takes values from 0 to encoderNumSteps
  if(derivFromUpwards <= 1){
    return 100;
  } else if(derivFromUpwards <= 5){
    return 50;
  } else if(derivFromUpwards <= 10){
    return 20;
  } 
  return 10;
}

