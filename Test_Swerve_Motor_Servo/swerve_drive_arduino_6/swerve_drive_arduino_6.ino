#include <Arduino.h>
#include <SMS_STS.h>

#define BAUD_RATE 115200

// Motor Pins
#define MOTOR1_PWM 4
#define MOTOR1_DIR 5
#define MOTOR2_PWM 6
#define MOTOR2_DIR 7
#define MOTOR3_PWM 8
#define MOTOR3_DIR 9
#define MOTOR4_PWM 10
#define MOTOR4_DIR 11

#define COMMAND_TIMEOUT_MS 9000
#define SAFETY_TIMEOUT_MS 18000
#define SERVO_DELAY_MS 1000

SMS_STS st, st2;

struct __attribute__((packed)) SwerveData {
  float motor1, motor2, motor3, motor4;
  float servo1, servo2, servo3, servo4;
  int32_t checksum;
};

SwerveData receivedData;
SwerveData nextData;
bool hasNewCommand = false;
bool isTimedOut = false;
bool firstTimeoutOccurred = false;

unsigned long lastUpdateTime = 0;
unsigned long servoStartTime = 0;

enum SystemState { IDLE, WAITING_SERVO, MOVING_MOTORS };
SystemState currentState = IDLE;

void controlMotor(int pwmPin, int dirPin, float speed, int motorIndex) {
  const int MIN_PWM = 10;
  const int START_PWM = 130;
  const int RAMP_TIME_MS = 400;
  const float DEAD_ZONE = 0.1f;

  static bool started[4] = {false};
  static unsigned long startTime[4] = {0};

  // Dead Zone: หยุดมอเตอร์ถ้าความเร็วต่ำมาก
  if (fabs(speed) < DEAD_ZONE) {
    analogWrite(pwmPin, 0);
    started[motorIndex] = false;
    return;
  }

  // แปลงความเร็วจาก -10 → 10 ไปเป็น -150 → 150
  int rawPWM = map(speed, -10, 10, -150, 150);
  rawPWM = constrain(rawPWM, -150, 150);

  // ถ้ายังไม่ได้เริ่ม เคลื่อนที่ ให้เริ่มที่ START_PWM
  if (!started[motorIndex]) {
    rawPWM = (rawPWM > 0) ? START_PWM : -START_PWM;
    startTime[motorIndex] = millis();
    started[motorIndex] = true;
    Serial.print("Motor"); Serial.print(motorIndex + 1);
    Serial.println(": START BOOST");
  } else {
    // ค่อย ๆ ลดลงจาก START_PWM → targetPWM
    unsigned long elapsed = millis() - startTime[motorIndex];
    if (elapsed < RAMP_TIME_MS) {
      float progress = (float)elapsed / RAMP_TIME_MS;
      int targetPWM = map(speed, -10, 10, -150, 150);
      int rampPWM = (int)(START_PWM + (abs(targetPWM) - START_PWM) * progress);
      rampPWM = constrain(rampPWM, MIN_PWM, 150);
      rawPWM = (rawPWM > 0) ? rampPWM : -rampPWM;
    }
  }

  digitalWrite(dirPin, rawPWM >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(rawPWM));
}


void controlServo(int id, float angle) {
  int angleint = angle * 100;
  int pos = map(angleint, -157, 157, 0, 4095);
  int speed = 2500;
  int torque = 25;
  st.WritePosEx(id, pos, speed, torque);
}

void stopAllMotors() {
  controlMotor(MOTOR1_PWM, MOTOR1_DIR, 0, 0);
  controlMotor(MOTOR2_PWM, MOTOR2_DIR, 0, 1);
  controlMotor(MOTOR3_PWM, MOTOR3_DIR, 0, 2);
  controlMotor(MOTOR4_PWM, MOTOR4_DIR, 0, 3);
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial2.begin(1000000);
  st.pSerial = &Serial2;

  Serial.println("✅ Arduino Mega Ready");

  pinMode(MOTOR1_PWM, OUTPUT); pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT); pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT); pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT); pinMode(MOTOR4_DIR, OUTPUT);

  int angleini = 180;
  int degree = map(angleini, 0, 360, 0, 4095);
  st.WritePosEx(1, degree, 1500, 50);
  st.WritePosEx(2, degree, 1500, 50);
  st.WritePosEx(3, degree, 1500, 50);
  st.WritePosEx(4, degree, 1500, 50);
  Serial.println("Servo Set 90 Degree");
  delay(2000);

  stopAllMotors();
}

void loop() {
  if (Serial.available() > 100) {
    Serial.println("⚠️ Overflow! Clearing Serial buffer...");
    while (Serial.available()) Serial.read();
    currentState = IDLE;
    return;
  }

  if (Serial.available() >= sizeof(SwerveData)) {
    Serial.readBytes((uint8_t*)&nextData, sizeof(SwerveData));
    while (Serial.available()) Serial.read();
    lastUpdateTime = millis();
    isTimedOut = false;
    firstTimeoutOccurred = false;

    if (currentState == IDLE) {
      receivedData = nextData;
      hasNewCommand = false;
      servoStartTime = millis();
      currentState = WAITING_SERVO;
    } else {
      hasNewCommand = true;
    }
  }

  if (!isTimedOut) {
    if (millis() - lastUpdateTime > COMMAND_TIMEOUT_MS) {
      if (!firstTimeoutOccurred) {
        stopAllMotors();
        Serial.println("⚠️ Motor timeout (temporary stop)");
        firstTimeoutOccurred = true;
      }
      if (millis() - lastUpdateTime > SAFETY_TIMEOUT_MS) {
        currentState = IDLE;
        isTimedOut = true;
        firstTimeoutOccurred = false;
        Serial.println("🛑 FULL SYSTEM TIMEOUT (Safety engaged)");
      }
    }
  }

  switch (currentState) {
    case WAITING_SERVO: {
      float delta = fabs(receivedData.servo1 - nextData.servo1) +
                    fabs(receivedData.servo2 - nextData.servo2) +
                    fabs(receivedData.servo3 - nextData.servo3) +
                    fabs(receivedData.servo4 - nextData.servo4);
      int effectiveDelay = (delta < 0.1) ? 100 : SERVO_DELAY_MS;

      if (millis() - servoStartTime >= effectiveDelay) {
        controlServo(1, receivedData.servo1);
        controlServo(2, receivedData.servo2);
        controlServo(3, receivedData.servo3);
        controlServo(4, receivedData.servo4);
        currentState = MOVING_MOTORS;
      }
      break;
    }

    case MOVING_MOTORS:
      controlMotor(MOTOR1_PWM, MOTOR1_DIR, receivedData.motor1, 0);
      controlMotor(MOTOR2_PWM, MOTOR2_DIR, receivedData.motor2, 1);
      controlMotor(MOTOR3_PWM, MOTOR3_DIR, receivedData.motor3, 2);
      controlMotor(MOTOR4_PWM, MOTOR4_DIR, -receivedData.motor4, 3);

      if (hasNewCommand) {
        receivedData = nextData;
        hasNewCommand = false;
        servoStartTime = millis();
        currentState = WAITING_SERVO;
        Serial.println("⚡ Interrupted! Executing new command");
      } else {
        currentState = IDLE;
        Serial.println("✅ Done. No queued command.");
      }
      break;

    case IDLE:
      break;
  }
}
