void setup() {
  //Serial.begin(12000000);
  Serial.begin(460800);
  while(!Serial.available()){

  }
  String msg = Serial.readStringUntil('\n');
  Serial.printf("Start Signal received: %s\n", msg);
}

int count = 0;
void loop() {
  //char message[] = "This is a test message sent via Serial from Teensy";
  char message[] = "TTTT";

  long start = micros();
  Serial.printf("%s %i\n", message, count);
  Serial.send_now();
  while(!Serial.available() && (micros() - start < 500000)){
    // Wait for response of time out after 500 milliseconds
  }
  String response = Serial.readStringUntil('\n');
  long end = micros();

  String expected = String(message) + " " + String(count);
  if(response == expected){
    Serial.printf("Latency: %i Microseconds\n", end - start);
    Serial.printf("Received Message: %s\n", response.c_str());
  } else{
    Serial.printf("Invalid Message: %s instead of %s\n", response.c_str(), expected.c_str());
  }
  count++;
  delay(1000);
}
