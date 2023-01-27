#include <Encoder.h>

#define FeedbackSerial Serial1

constexpr unsigned pinEncoderA = 2;
constexpr unsigned pinEncoderB = 3;
unsigned long samplingPeriodMillis = 50;
constexpr int encoderNumSteps = 2400;

int previousEncoderValue = 0;

float compareValue = 0;

unsigned long lastSampleTime = 0;
long sequenceNumber = 1;
long lastReceivedSequenceNumber = 0;
long packetsLostTotal = 0;


Encoder encoder(pinEncoderA, pinEncoderB);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  delay(5000);

  Serial.setTimeout(20);

  FeedbackSerial.begin(115200);
  FeedbackSerial.setTimeout(20);
}

void loop() {
  sendSampleIfDelayOver();
  checkAndHandleFeedback();
}

void sendSampleIfDelayOver(){
  unsigned long currentTime = millis();
  if(lastSampleTime + samplingPeriodMillis <= currentTime){
    // The sample value string is of the following form:
    // S:encoderValue;samplingPeriodMillis;sequenceNumber;currentTime;\n
    // for example
    // S:-1204;50;1234;62345234;\n
    int encoderValue = encoder.read();  
    Serial.printf("S:%i;%u;%i;%u;\n", encoderValue, samplingPeriodMillis, sequenceNumber, currentTime);
    Serial.send_now();
    lastSampleTime = currentTime;
    sequenceNumber++;

    // Must be updated AFTER transmission, as the value just sent still has the
    // old sampling period:
    samplingPeriodMillis = angleToSamplePeriod(encoderValue);
    previousEncoderValue = encoderValue;
  }
}

void checkAndHandleFeedback(){
    if(FeedbackSerial.available() > 0){
    // The Feedback is of the following form:
    // sequenceNumber;Timestamp;
    // for example
    // FB:123;34523890;\n


    // The logging information sent to the sender site Linux client has the following form:
    // packetsLostTotal;latencyMillis;\n
    // for example
    // FB:55;13;\n
    if(FeedbackSerial.readStringUntil(':').startsWith("FB")){
      long receivedSequenceNumber = FeedbackSerial.readStringUntil(';').toInt();
      long timestamp = FeedbackSerial.readStringUntil(';').toInt();
      FeedbackSerial.read(); // read '\n'
      long latency = millis() - timestamp;

      if(lastReceivedSequenceNumber > 0){ // don't count packet loss for first received packet
        packetsLostTotal += max(0, receivedSequenceNumber - lastReceivedSequenceNumber - 1);
      }

      lastReceivedSequenceNumber = receivedSequenceNumber;

      Serial.printf("FB:%i;%i;\n", packetsLostTotal, latency);    
      Serial.send_now();
    }
  }
}


int angleToSamplePeriod(int encoderValue){
  // int changeFromPrevious = abs(previousEncoderValue - encoderValue);

  // float alpha = 0.5 * ((float) samplingPeriodMillis / 100.0);
  // compareValue = alpha * (float) changeFromPrevious + (1-alpha) * compareValue;

  // if(compareValue <= 0.5){
  //   return 100;
  // } else if(compareValue <= 1.0){
  //   return 50;
  // } else if(compareValue <= 3.0){
  //   return 20;
  // } 
  // return 10;
  return 50;
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

