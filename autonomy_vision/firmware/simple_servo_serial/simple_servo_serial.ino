#include <Servo.h>

Servo panServo;

// -------- CONFIG --------
const int PAN_SERVO_PIN = 3;

const int PAN_MIN_DEG = 0;
const int PAN_MAX_DEG = 180;

const int SERVO_STEP_DELAY_MS = 10;  // smooth motion
const int SERVO_STEP_DEG = 1;         // degrees per step
// ------------------------

int currentAngle = 90;

void setup() {
  Serial.begin(115200);
  panServo.attach(PAN_SERVO_PIN);

  // Move to a known starting position
  panServo.write(currentAngle);
  delay(500);
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.startsWith("PAN")) {
    int target = line.substring(3).toInt();

    target = constrain(target, PAN_MIN_DEG, PAN_MAX_DEG);

    moveServoBlocking(target);

    Serial.println("OK");
  }
}

void moveServoBlocking(int target) {
  if (target == currentAngle) return;

  int step = (target > currentAngle) ? SERVO_STEP_DEG : -SERVO_STEP_DEG;

  while (currentAngle != target) {
    currentAngle += step;

    // Prevent overshoot
    if ((step > 0 && currentAngle > target) ||
        (step < 0 && currentAngle < target)) {
      currentAngle = target;
    }

    panServo.write(currentAngle);
    delay(SERVO_STEP_DELAY_MS);
  }
}
