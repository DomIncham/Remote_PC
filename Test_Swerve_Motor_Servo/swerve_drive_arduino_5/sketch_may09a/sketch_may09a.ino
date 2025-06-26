
#include <Arduino.h>
#include <SMS_STS.h>

#define BAUD_RATE 115200

// Motor pins
#define MOTOR1_PWM 4
#define MOTOR1_DIR 5
#define MOTOR2_PWM 6
#define MOTOR2_DIR 7
#define MOTOR3_PWM 8
#define MOTOR3_DIR 9
#define MOTOR4_PWM 10
#define MOTOR4_DIR 11

#define SERVO_DELAY_MS 100
#define COMMAND_TIMEOUT_MS 9000
#define SAFETY_TIMEOUT_MS 18000

SMS_STS st;

struct __attribute__((packed)) SwerveData {
  float motor1, motor2, motor3, motor4; 
  float servo1, servo2, servo3, servo4;
  int32_t checksum;
};

SwerveData receivedData, nextData;
bool hasNewCommand = false;
bool isTimedOut = false;
bool firstTimeoutOccurred = false;

unsigned long lastUpdateTime = 0;
unsigned long servoStartTime = 0;

float lastServo1 = 0, lastServo2 = 0, lastServo3 = 0, lastServo4 = 0;
bool servoChanged = false;

enum SystemState { IDLE, WAITING_SERVO, MOVING_MOTORS };
SystemState currentState = IDLE;

void controlMotor(int pwmPin, int dirPin, float speed) {
  int pwm = map(speed, -10.0, 10.0, -150, 150);
  digitalWrite(dirPin, pwm > 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(pwm));
}

void controlServo(int id, float angle) {
  int angleint = angle * 100;
  int pos = map(angleint, -157, 157, 0, 4095);
  st.WritePosEx(id, pos, 2500, 150);
}

void stopAllMotors() {
  controlMotor(MOTOR1_PWM, MOTOR1_DIR, 0);
  controlMotor(MOTOR2_PWM, MOTOR2_DIR, 0);
  controlMotor(MOTOR3_PWM, MOTOR3_DIR, 0);
  controlMotor(MOTOR4_PWM, MOTOR4_DIR, 0);
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial2.begin(1000000);
  st.pSerial = &Serial2;

  pinMode(MOTOR1_PWM, OUTPUT); pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT); pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT); pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT); pinMode(MOTOR4_DIR, OUTPUT);

  st.WritePosEx(1, 2048, 1500, 50);
  st.WritePosEx(2, 2048, 1500, 50);
  st.WritePosEx(3, 2048, 1500, 50);
  st.WritePosEx(4, 2048, 1500, 50);

  delay(1500);
  stopAllMotors();
}

void loop() {
  if (Serial.available() > 100) {
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

    receivedData = nextData;
    hasNewCommand = false;

    servoChanged =
      receivedData.servo1 != lastServo1 ||
      receivedData.servo2 != lastServo2 ||
      receivedData.servo3 != lastServo3 ||
      receivedData.servo4 != lastServo4;

    if (servoChanged) {
      servoStartTime = millis();
      currentState = WAITING_SERVO;
    } else {
      currentState = MOVING_MOTORS;
    }
  }

  if (!isTimedOut) {
    if (millis() - lastUpdateTime > COMMAND_TIMEOUT_MS) {
      if (!firstTimeoutOccurred) {
        stopAllMotors();
        firstTimeoutOccurred = true;
      }
      if (millis() - lastUpdateTime > SAFETY_TIMEOUT_MS) {
        currentState = IDLE;
        isTimedOut = true;
        firstTimeoutOccurred = false;
      }
    }
  }

  switch (currentState) {
    case WAITING_SERVO:
      controlServo(1, receivedData.servo1);
      controlServo(2, receivedData.servo2);
      controlServo(3, receivedData.servo3);
      controlServo(4, receivedData.servo4);

      if (millis() - servoStartTime >= SERVO_DELAY_MS) {
        lastServo1 = receivedData.servo1;
        lastServo2 = receivedData.servo2;
        lastServo3 = receivedData.servo3;
        lastServo4 = receivedData.servo4;
        currentState = MOVING_MOTORS;
      }
      break;

    case MOVING_MOTORS:
      controlMotor(MOTOR1_PWM, MOTOR1_DIR, receivedData.motor1);
      controlMotor(MOTOR2_PWM, MOTOR2_DIR, -receivedData.motor2);
      controlMotor(MOTOR3_PWM, MOTOR3_DIR, receivedData.motor3);
      controlMotor(MOTOR4_PWM, MOTOR4_DIR, receivedData.motor4);
      currentState = IDLE;
      break;

    case IDLE:
      // do nothing
      break;
  }
}
