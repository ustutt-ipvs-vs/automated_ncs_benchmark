#include <Encoder.h>

#define FeedbackSerial Serial1

constexpr unsigned pinEncoderA = 2;
constexpr unsigned pinEncoderB = 3;
unsigned long transmissionPeriodMillis = 50;
unsigned long sensorPeekPeriodMillis = 10;
constexpr int encoderNumSteps = 2400;

int previousEncoderValue;

float compareValue = 0;

unsigned long lastTransmissionTime = 0;
unsigned long lastSensorPeekTime = 0;

long sequenceNumber = 1;
long lastReceivedSequenceNumber = 0;
long packetsLostTotal = 0;

#define HISTORY_SIZE 100
int samplesHistory[HISTORY_SIZE];  // acts as a ring buffer
int historyPosition = 0;


Encoder encoder(pinEncoderA, pinEncoderB);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  delay(5000);

  Serial.setTimeout(20);

  FeedbackSerial.begin(115200);
  FeedbackSerial.setTimeout(20);
}

void loop() {
  calculateTransmissionPeriod();
  sendSampleIfDelayOver();

  checkAndHandleFeedback();
}

void calculateTransmissionPeriod(){
  unsigned long currentTime = millis();
  if(lastSensorPeekTime + sensorPeekPeriodMillis <= currentTime){
    int currentEncoderValue = encoder.read();
    lastSensorPeekTime = currentTime;

    compareValue = getHistoryMaxChange(currentEncoderValue);

    samplesHistory[historyPosition] = currentEncoderValue;
    historyPosition = (historyPosition + 1) % HISTORY_SIZE;

    previousEncoderValue = currentEncoderValue;

    if(compareValue <= 1){
      transmissionPeriodMillis = 100;
    } else if(compareValue <= 2){
      transmissionPeriodMillis = 90;
    } else if(compareValue <= 3){
      transmissionPeriodMillis = 80;
    } else if(compareValue <= 4){
      transmissionPeriodMillis = 70;
    } else if(compareValue <= 5){
      transmissionPeriodMillis = 60;
    } else if(compareValue <= 6){
      transmissionPeriodMillis = 50;
    } else if(compareValue <= 7){
      transmissionPeriodMillis = 40;
    } else if(compareValue <= 8){
      transmissionPeriodMillis = 30;
    } else if(compareValue <= 9){
      transmissionPeriodMillis = 20;
    } else {
      transmissionPeriodMillis = 10;
    }
  }
}

void sendSampleIfDelayOver(){
  unsigned long currentTime = millis();
  if(lastTransmissionTime + transmissionPeriodMillis <= currentTime){
    // The sample value string is of the following form:
    // S:encoderValue;samplingPeriodMillis;sequenceNumber;currentTime;\n
    // for example
    // S:-1204;50;1234;62345234;\n
    Serial.printf("S:%i;%u;%i;%u;\n", encoder.read(), transmissionPeriodMillis, sequenceNumber, currentTime);
    Serial.send_now();
    lastTransmissionTime = currentTime;
    sequenceNumber++;
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


int getHistoryMaxChange(int currentValue){
  int maxChange = abs(samplesHistory[0] - currentValue);
  for(int i=1; i < HISTORY_SIZE; i++){
    int change = abs(samplesHistory[i] - currentValue);
    if(change > maxChange){
      maxChange = change;
    }
  }
  return maxChange;
}


