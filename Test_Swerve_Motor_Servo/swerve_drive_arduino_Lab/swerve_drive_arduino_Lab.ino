#include <Arduino.h>
#include <SMS_STS.h>

#define BAUD_RATE 115200

// มอเตอร์
#define MOTOR1_PWM 4
#define MOTOR1_DIR 5
#define MOTOR2_PWM 6
#define MOTOR2_DIR 7
#define MOTOR3_PWM 8
#define MOTOR3_DIR 9
#define MOTOR4_PWM 10
#define MOTOR4_DIR 11

// เวลา
#define COMMAND_TIMEOUT_MS 8000
#define SAFETY_TIMEOUT_MS 16000
#define SERVO_DELAY_MS 1000
#define SERVO_SPEED 2600
#define SERVO_TORQUE 50
#define RAMP_DOWN_TIME_MS (SERVO_DELAY_MS * 0.8)

// สั่งหยุดเมื่อ speed < DEAD_ZONE
#define DEAD_ZONE 0.1f

SMS_STS st;

#pragma pack(push, 1)
struct SwerveData {
  float motor1, motor2, motor3, motor4;
  float servo1, servo2, servo3, servo4;
  int32_t checksum;
};
#pragma pack(pop)

SwerveData receivedData;
SwerveData nextData;

unsigned long lastUpdateTime = 0;
unsigned long servoStartTime = 0;
float servoProgress = 0.0;

bool isMoving[4] = {false};
unsigned long motorStartTime[4] = {0};
bool isTimedOut = false;
bool firstTimeoutOccurred = false;

void controlMotor(int pwmPin, int dirPin, float speed, int motorIndex) {
  const int MAX_PWM = 130;
  const int START_PWM = 130;
  const int MIN_PWM = 10;

  if (fabs(speed) < DEAD_ZONE) {
    if (isMoving[motorIndex]) {
      analogWrite(pwmPin, 0);
      isMoving[motorIndex] = false;
      Serial.print("Motor");
      Serial.print(motorIndex + 1);
      Serial.println(": STOPPED");
    }
    return;
  }

  int targetPWM = speed * MAX_PWM;
  targetPWM = constrain(targetPWM, -MAX_PWM, MAX_PWM);

  if (!isMoving[motorIndex]) {
    targetPWM = (targetPWM > 0) ? START_PWM : -START_PWM;
    isMoving[motorIndex] = true;
    motorStartTime[motorIndex] = millis();
    Serial.print("Motor");
    Serial.print(motorIndex + 1);
    Serial.print(": STARTING @ PWM ");
    Serial.println(targetPWM);
  } else {
    float rampProgress = min(1.0, (millis() - motorStartTime[motorIndex]) / (float)RAMP_DOWN_TIME_MS);
    int basePWM = abs(targetPWM);
    if (basePWM < MIN_PWM) basePWM = MIN_PWM;
    int currentPWM = START_PWM + (basePWM - START_PWM) * rampProgress;
    currentPWM = constrain(currentPWM, MIN_PWM, MAX_PWM);
    targetPWM = (targetPWM > 0) ? currentPWM : -currentPWM;
  }

  digitalWrite(dirPin, (targetPWM >= 0) ? HIGH : LOW);
  analogWrite(pwmPin, abs(targetPWM));
}

void controlServo(int id, float angle) {
  int angleInt = constrain(angle * 100, -157, 157);
  int pos = map(angleInt, -157, 157, 0, 4095);
  st.WritePosEx(id, pos, SERVO_SPEED, SERVO_TORQUE);
}

void stopAllMotors() {
  Serial.println("🛑 STOP ALL MOTORS");
  for (int pin : {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM}) {
    analogWrite(pin, 0);
  }
  for (int i = 0; i < 4; ++i) {
    isMoving[i] = false;
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial2.begin(1000000);
  st.pSerial = &Serial2;

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);

  Serial.println("🔧 Servo INIT");
  int centerPos = map(18000, 0, 36000, 0, 4095);
  for (int id = 1; id <= 4; id++) {
    st.WritePosEx(id, centerPos, SERVO_SPEED, SERVO_TORQUE);
  }

  delay(1500);
  stopAllMotors();
  Serial.println("🚀 Swerve Drive Ready!");
}

void loop() {
  // ตรวจสอบ buffer overflow
  if (Serial.available() > 100) {
    Serial.println("⚠️ BUFFER OVERFLOW! Clearing...");
    while (Serial.available()) Serial.read();
    stopAllMotors();
    return;
  }

  // รับคำสั่งใหม่
  if (Serial.available() >= sizeof(SwerveData)) {
    Serial.readBytes((uint8_t*)&nextData, sizeof(SwerveData));
    while (Serial.available()) Serial.read();

    lastUpdateTime = millis();
    servoStartTime = millis();
    receivedData = nextData;
    isTimedOut = false;
    firstTimeoutOccurred = false;

    Serial.print("📡 Motors:[");
    Serial.print(receivedData.motor1); Serial.print(", ");
    Serial.print(receivedData.motor2); Serial.print(", ");
    Serial.print(receivedData.motor3); Serial.print(", ");
    Serial.print(receivedData.motor4); Serial.print("] Servos:[");
    Serial.print(receivedData.servo1); Serial.print(", ");
    Serial.print(receivedData.servo2); Serial.print(", ");
    Serial.print(receivedData.servo3); Serial.print(", ");
    Serial.print(receivedData.servo4); Serial.println("]");

    controlServo(1, receivedData.servo1);
    controlServo(2, receivedData.servo2);
    controlServo(3, -receivedData.servo3);
    controlServo(4, -receivedData.servo4);
  }

  // คำนวณ ramp
  servoProgress = min(1.0, (millis() - servoStartTime) / (float)SERVO_DELAY_MS);
  float rampSpeed = servoProgress * servoProgress;

  controlMotor(MOTOR1_PWM, MOTOR1_DIR, receivedData.motor1 * rampSpeed, 0);
  controlMotor(MOTOR2_PWM, MOTOR2_DIR, receivedData.motor2 * rampSpeed, 1);
  controlMotor(MOTOR3_PWM, MOTOR3_DIR, receivedData.motor3 * rampSpeed, 2);
  controlMotor(MOTOR4_PWM, MOTOR4_DIR, -receivedData.motor4 * rampSpeed, 3);

  // ตรวจสอบ Timeout
  if (!isTimedOut && millis() - lastUpdateTime > COMMAND_TIMEOUT_MS) {
    if (!firstTimeoutOccurred) {
      stopAllMotors();
      Serial.print("⚠️ TIMEOUT ");
      Serial.print((millis() - lastUpdateTime) / 1000.0, 1);
      Serial.println("s");
      firstTimeoutOccurred = true;
    }

    if (millis() - lastUpdateTime > SAFETY_TIMEOUT_MS) {
      isTimedOut = true;
      firstTimeoutOccurred = false;
      Serial.println("🛑 SAFETY LOCK - No command for 16s");
    }
  }
}
