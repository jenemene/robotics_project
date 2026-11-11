// DRV8871 + Serial Monitor control with kick-start and fast stop
// Serial Monitor: 115200 baud, any line ending
const int IN1 = 5;
const int IN2 = 6;

const int KICK_PWM = 80;      // above breakaway (yours is ~45)
const int KICK_MS  = 40;      // kick length in ms, tune 20-80
const int REVERSE_PAUSE_MS = 100;  // settle time before reversing

const int STEP    = 20;       // ramp step between running speeds
const int STEP_MS = 5;        // ms per step -> full range in ~65 ms

const bool BRAKE_ON_ZERO = true;  // true = '0' brakes hard, false = coast

int current = 0;              // applied speed (-255..255)
int target  = 0;              // requested speed
unsigned long lastStep = 0;

char buf[16];
byte len = 0;
unsigned long lastChar = 0;

void applyMotor(int s) {
  current = constrain(s, -255, 255);
  if (current > 0)      { analogWrite(IN2, 0); analogWrite(IN1, current); }
  else if (current < 0) { analogWrite(IN1, 0); analogWrite(IN2, -current); }
  else                  { analogWrite(IN1, 0); analogWrite(IN2, 0); }   // coast
}

void brakeNow() {
  target = 0;
  current = 0;
  analogWrite(IN1, 255);
  analogWrite(IN2, 255);
}

void stopNow() {
  target = 0;
  if (BRAKE_ON_ZERO) brakeNow();
  else applyMotor(0);
}

void startWithKick(int t) {
  applyMotor(t > 0 ? KICK_PWM : -KICK_PWM);   // break it loose
  delay(KICK_MS);
  applyMotor(t);                              // drop straight to requested speed
}

void handleCommand(char *cmd) {
  Serial.print(F("Got: '")); Serial.print(cmd); Serial.println(F("'"));
  char c = cmd[0];

  if (c == 's' || c == 'S') {
    stopNow();
    Serial.println(F("STOP"));
  } else if (c == 'b' || c == 'B') {
    brakeNow();
    Serial.println(F("BRAKE"));
  } else if (c == '?') {
    Serial.print(F("current=")); Serial.print(current);
    Serial.print(F(" target="));  Serial.println(target);
  } else if (c == '-' || isDigit(c)) {
    int newTarget = constrain(atoi(cmd), -255, 255);

    if (newTarget == 0) {                       // stop immediately, no ramp
      stopNow();
      Serial.println(F("Target: 0 (stopped)"));
      return;
    }

    bool wasStopped = (current == 0);
    bool reversing  = !wasStopped && ((current > 0) != (newTarget > 0));
    target = newTarget;

    if (reversing) {                            // stop, settle, kick other way
      applyMotor(0);
      delay(REVERSE_PAUSE_MS);
      startWithKick(target);
    } else if (wasStopped) {                    // from standstill: kick
      startWithKick(target);
    }
    // same direction while running: the ramp in loop() handles it

    Serial.print(F("Target: ")); Serial.println(target);
  } else {
    stopNow();
    Serial.println(F("Unknown -> stopped"));
  }
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  applyMotor(0);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Serial.println(F("Ready: -255..255, 0/s=stop, b=brake, ?=status"));
}

void loop() {
  // Read commands: end on CR/LF, or after 100 ms of no input
  while (Serial.available()) {
    char ch = Serial.read();
    lastChar = millis();
    if (ch == '\n' || ch == '\r') {
      if (len) { buf[len] = 0; handleCommand(buf); len = 0; }
    } else if (ch != ' ' && len < sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }
  if (len && millis() - lastChar > 100) { buf[len] = 0; handleCommand(buf); len = 0; }

  // Ramp between running speeds in the same direction (e.g. 150 -> 40)
  if (current != target && millis() - lastStep >= STEP_MS) {
    lastStep = millis();
    applyMotor(current < target ? min(current + STEP, target)
                                : max(current - STEP, target));
  }
}