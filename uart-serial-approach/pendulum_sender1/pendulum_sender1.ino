#include <Encoder.h>

constexpr unsigned pinEncoderA = 2;
constexpr unsigned pinEncoderB = 3;
constexpr int samplingPeriodMillis = 50;

Encoder encoder(pinEncoderA, pinEncoderB);

void setup() {
  //Serial.begin(12000000);
  Serial.begin(460800);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  int encoderValue = encoder.read();
  Serial.printf("%i;\n", encoderValue);
  delay(samplingPeriodMillis);
}
