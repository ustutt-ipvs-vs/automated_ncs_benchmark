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

int historySize = 0;
int* samplesHistory; // Acts as ring buffer
#define NUM_SAMPLING_PERIODS 10
int samplingPeriodsMillis[NUM_SAMPLING_PERIODS];
int historyPosition = 0;

double angleSpeed;

Encoder encoder(pinEncoderA, pinEncoderB);
Stream* sensorValueSerial;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  delay(5000);

  Serial.setTimeout(20);

  sensorValueSerial = &Serial;

  readInitializationValues();

  FeedbackSerial.begin(115200);
  FeedbackSerial.setTimeout(20);
}

void readInitializationValues(){
  // Expected Format: 
  // H:historySize;period1;period2;period3;period4;period5;period6;period7;period8;period9;period10;\n
  bool receivedHistorySize = false;
  while(!receivedHistorySize){
    Serial.println("READY");
    delay(500);

    if(Serial.available() > 0){
      if(Serial.readStringUntil(':').startsWith("H")){
        historySize = Serial.readStringUntil(';').toInt();
        for(int i=0; i<NUM_SAMPLING_PERIODS; i++){
          samplingPeriodsMillis[i] = Serial.readStringUntil(';').toInt();
        }
        receivedHistorySize = true;
        samplesHistory = new int[historySize];

        // Echo received values to computer for validation:
        Serial.printf("Received history size: %i\n", historySize);
        Serial.print("Received Sampling Periods: ");
        for(int i=0; i<NUM_SAMPLING_PERIODS; i++){
          Serial.printf("%i, ", samplingPeriodsMillis[i]);
        }
        Serial.println();
      }
      Serial.read(); // Remove \n
    }
  }
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
    historyPosition = (historyPosition + 1) % historySize;

    previousEncoderValue = currentEncoderValue;

    if(compareValue <= 1){
      transmissionPeriodMillis = samplingPeriodsMillis[0];
    } else if(compareValue <= 2){
      transmissionPeriodMillis = samplingPeriodsMillis[1];
    } else if(compareValue <= 3){
      transmissionPeriodMillis = samplingPeriodsMillis[2];
    } else if(compareValue <= 4){
      transmissionPeriodMillis = samplingPeriodsMillis[3];
    } else if(compareValue <= 5){
      transmissionPeriodMillis = samplingPeriodsMillis[4];
    } else if(compareValue <= 6){
      transmissionPeriodMillis = samplingPeriodsMillis[5];
    } else if(compareValue <= 7){
      transmissionPeriodMillis = samplingPeriodsMillis[6];
    } else if(compareValue <= 8){
      transmissionPeriodMillis = samplingPeriodsMillis[7];
    } else if(compareValue <= 9){
      transmissionPeriodMillis = samplingPeriodsMillis[8];
    } else {
      transmissionPeriodMillis = samplingPeriodsMillis[9];
    }
  }
}

void sendSampleIfDelayOver(){
  unsigned long currentTime = millis();
  float angularVelocity = getAngularVelocity();
  if(lastTransmissionTime + transmissionPeriodMillis <= currentTime){
    // The sample value string is of the following form:
    // S:encoderValue;samplingPeriodMillis;sequenceNumber;currentTime;angularVelocity;\n
    // for example
    // S:-1204;50;1234;62345234;-1.308997\n
    sensorValueSerial->printf("S:%i;%u;%i;%u;%f;\n", encoder.read(), transmissionPeriodMillis, sequenceNumber, currentTime, angularVelocity);
    Serial.send_now();
    lastTransmissionTime = currentTime;
    sequenceNumber++;
  }
}

float getAngularVelocity(){
  int currentSample = samplesHistory[(historyPosition - 1 + historySize) % historySize];
  int previousSample = samplesHistory[(historyPosition - 2 + historySize) % historySize];
  float angleChange = (((float) currentSample - (float) previousSample) / (float) encoderNumSteps) * 2.f * PI;
  float angularVelocity = angleChange / ((float) sensorPeekPeriodMillis / 1000.f);
  return angularVelocity;
}

void checkAndHandleFeedback(){
    const int prefixLength = 3; // Length of string "FB:"
    if(FeedbackSerial.available() >= prefixLength){
    // The Feedback is of the following form:
    // sequenceNumber;Timestamp;
    // for example
    // FB:123;34523890;\n

    int firstChar = FeedbackSerial.read();
    int secondChar = FeedbackSerial.read();
    int thirdChar = FeedbackSerial.read();


    // The logging information sent to the sender site Linux client has the following form:
    // packetsLostTotal;latencyMillis;\n
    // for example
    // FB:55;13;\n
    // 
    // The signals for start and end of swing up have the following formats:
    // SS;\n
    // SE;\n
    if(firstChar == 'F' && secondChar == 'B' && thirdChar == ':'){ // Feedback packet
      long receivedSequenceNumber = FeedbackSerial.readStringUntil(';').toInt();
      long timestamp = FeedbackSerial.readStringUntil(';').toInt();
      FeedbackSerial.readStringUntil('\n'); // read '\n'
      long latency = millis() - timestamp;

      if(lastReceivedSequenceNumber > 0){ // don't count packet loss for first received packet
        packetsLostTotal += max(0, receivedSequenceNumber - lastReceivedSequenceNumber - 1);
      }

      lastReceivedSequenceNumber = receivedSequenceNumber;

      Serial.printf("FB:%i;%i;\n", packetsLostTotal, latency);    
      Serial.send_now();
    } else if (firstChar == 'S' && secondChar == 'S' && thirdChar == ';'){
      FeedbackSerial.readStringUntil('\n'); // read '\n'
      sensorValueSerial = &FeedbackSerial;
      Serial.println("Starting to send sensor values through feedback link for swing-up.");
    } else if (firstChar == 'S' && secondChar == 'E' && thirdChar == ';'){
      FeedbackSerial.readStringUntil('\n'); // read '\n'
      sensorValueSerial = &Serial; // USB serial
      Serial.println("Swing-up complete. Sending sensor values through USB serial again.");
    } else if (firstChar == 'C' && secondChar == 'T' && thirdChar == ':'){
      // Forward control message (prefix 'CT:') to linux  sender:
      Serial.printf("CT:%s\n", FeedbackSerial.readStringUntil('\n').c_str()); // read '\n'
    } 
  }
}


int getHistoryMaxChange(int currentValue){
  int maxChange = abs(samplesHistory[0] - currentValue);
  for(int i=1; i < historySize; i++){
    int change = abs(samplesHistory[i] - currentValue);
    if(change > maxChange){
      maxChange = change;
    }
  }
  return maxChange;
}


