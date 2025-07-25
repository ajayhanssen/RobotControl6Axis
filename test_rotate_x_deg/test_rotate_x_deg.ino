#define STEP_PIN 2
#define DIR_PIN 3

const float stepAngle = 1.8;  // degrees per full step

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
}

// Rotate by a specific angle (positive = CW, negative = CCW)
void rotateDegrees(float angle) {
  if (angle == 0) return;

  bool dir = (angle > 0);
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);

  int steps = abs(angle) / stepAngle;
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }
}

void loop() {
  rotateDegrees(90);   // rotate +90°
  delay(1000);
  rotateDegrees(-90);  // rotate -90°
  delay(1000);
}