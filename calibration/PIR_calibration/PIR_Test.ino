const int PIR_PIN = 2;
const int OUT_PIN = 13;              // LED (or signal to the robot)
const unsigned long HOLD_MS = 1000;  // stay HIGH 5 s after last motion

unsigned long lastMotion = 0;

void setup() {
  pinMode(PIR_PIN, INPUT);
  pinMode(OUT_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (digitalRead(PIR_PIN) == HIGH)
    lastMotion = millis();

  bool person = (millis() - lastMotion) < HOLD_MS;
  digitalWrite(OUT_PIN, person);
  Serial.println(person ? "PERSON" : "-");
  delay(100);
}