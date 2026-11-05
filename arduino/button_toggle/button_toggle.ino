const int BTN = 2;
const int LED = 4;   // external LED circuit on digital pin 4
bool state = false;
bool lastReading = HIGH;
unsigned long lastChange = 0;

void setup() {
  pinMode(BTN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // 1. Commands from the computer ("ON" or "OFF" + newline)
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "ON" || cmd == "OFF") {
      state = (cmd == "ON");
      Serial.println(state ? "ACK:ON" : "ACK:OFF");
    }
  }

  // 2. Button with 50 ms debounce: just report the press
  bool reading = digitalRead(BTN);
  if (reading != lastReading && millis() - lastChange > 50) {
    lastChange = millis();
    if (reading == LOW) {        // just pressed
      Serial.println("PRESS");
    }
  }
  lastReading = reading;

  digitalWrite(LED, state);
}