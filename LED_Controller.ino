
void setup() {
  pinMode (LED_BUILTIN, OUTPUT);
  // initialize serial:
  Serial.begin(115200);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    Serial.read();
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
