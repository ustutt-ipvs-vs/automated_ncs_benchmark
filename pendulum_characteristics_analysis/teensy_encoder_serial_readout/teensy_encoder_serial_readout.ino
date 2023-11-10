/**
* This script reads out the rotary encoder every millisecond and prints the value to the 
* serial interface together with the system time in microseconds.
* 
* Each printed line has the following format:
* timeMicros;encoderValue\n
*/

#include <Encoder.h>

unsigned long timeLastSample = 0;
constexpr unsigned pinEncoderA = 2;
constexpr unsigned pinEncoderB = 3;
Encoder encoder(pinEncoderA, pinEncoderB);

void setup() {

}

void loop() {
  unsigned long currentTimeMicros = micros();
  if(currentTimeMicros - timeLastSample >= 1000){
    int encoderValue = encoder.read();
    Serial.printf("%d;%d\n", currentTimeMicros, encoderValue);
    timeLastSample = currentTimeMicros;
  }
}
