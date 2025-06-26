#include <Arduino.h>
#include <SMS_STS.h>
#include <PinChangeInterrupt.h>

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

#define SERVO_DELAY_MS 0
#define COMMAND_TIMEOUT_MS 9000
#define SAFETY_TIMEOUT_MS 18000

// Encoder pins
const uint8_t encoderPinsA[4] = {A8, A10, A12, A14};
const uint8_t encoderPinsB[4] = {A9, A11, A13, A15};
volatile long encoderCount[4] = {0};
long prevEncoderCount[4] = {0};
float totalDistance[4] = {0};
const int pulsesPerRev = 360;
const float wheelRadius = 0.12;
const float distancePerPulse = 2 * PI * wheelRadius / pulsesPerRev;
unsigned long lastDistancePrint = 0;

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

// === Encoder ISR Functions ===
void encoderISR(int i) {
  encoderCount[i] += (digitalRead(encoderPinsA[i]) == digitalRead(encoderPinsB[i])) ? 1 : -1;
}
void encoderISR0() { encoderISR(0); }
void encoderISR1() { encoderISR(1); }
void encoderISR2() { encoderISR(2); }
void encoderISR3() { encoderISR(3); }

void setupEncoders() {
  for (int i = 0; i < 4; i++) {
    pinMode(encoderPinsA[i], INPUT_PULLUP);
    pinMode(encoderPinsB[i], INPUT_PULLUP);
  }
  attachPCINT(digitalPinToPCINT(encoderPinsA[0]), encoderISR0, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinsA[1]), encoderISR1, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinsA[2]), encoderISR2, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPinsA[3]), encoderISR3, CHANGE);
}

void controlMotor(int pwmPin, int dirPin, float speed) {
  int pwm = map(speed, -10.0, 10.0, -150, 150);
  digitalWrite(dirPin, pwm > 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(pwm));
}

void controlServo(int id, float angle) {
  int angleint = angle * 100;
  int pos = map(angleint, -157, 157, 0, 4095);
  st.WritePosEx(id, pos, 3000, 150);
}

void stopAllMotors() {
  controlMotor(MOTOR1_PWM, MOTOR1_DIR, 0);
  controlMotor(MOTOR2_PWM, MOTOR2_DIR, 0);
  controlMotor(MOTOR3_PWM, MOTOR3_DIR, 0);
  controlMotor(MOTOR4_PWM, MOTOR4_DIR, 0);
}

void setup() {
  Serial.flush();
  Serial2.flush();
  Serial.begin(BAUD_RATE);
  Serial2.begin(1000000);
  st.pSerial = &Serial2;

  pinMode(MOTOR1_PWM, OUTPUT); pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT); pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT); pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT); pinMode(MOTOR4_DIR, OUTPUT);

  st.WritePosEx(1, 2048, 1500, 50);
  st.WritePosEx(2, 2048, 1500, 50);
  //st.WritePosEx(3, 2048, 1500, 50);
  //st.WritePosEx(4, 2048, 1500, 50);

  setupEncoders();
  delay(1500);
  stopAllMotors();
  lastDistancePrint = millis();
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
      //receivedData.servo1 != lastServo1 ||
      receivedData.servo2 != lastServo2 ||
      receivedData.servo3 != lastServo3; //||
      //receivedData.servo4 != lastServo4;

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
      //controlServo(3, receivedData.servo3);
      //controlServo(4, receivedData.servo4);
      controlServo(1, receivedData.servo1);
      controlServo(2, receivedData.servo2);

      if (millis() - servoStartTime >= SERVO_DELAY_MS) {
        lastServo1 = receivedData.servo1;
        lastServo2 = receivedData.servo2;
        //lastServo3 = receivedData.servo3;
        //lastServo4 = receivedData.servo4;
        currentState = MOVING_MOTORS;
      }
      break;

    case MOVING_MOTORS:
      /*float motor2_cmd = receivedData.motor2;
      if (abs(motor2_cmd) > 0.01) {
        motor2_cmd += (motor2_cmd > 0) ? 1.0 : -1.0;
      }
      controlMotor(MOTOR2_PWM, MOTOR2_DIR, -motor2_cmd);*/
      controlMotor(MOTOR2_PWM, MOTOR2_DIR, receivedData.motor2);
      controlMotor(MOTOR1_PWM, MOTOR1_DIR, -receivedData.motor1);
      currentState = IDLE;
      break;
  }

  // === Encoder Distance Display (every 5 seconds) ===
  if (millis() - lastDistancePrint >= 5000) {
    lastDistancePrint = millis();
    for (int i = 0; i < 4; i++) {
      long delta = encoderCount[i] - prevEncoderCount[i];
      prevEncoderCount[i] = encoderCount[i];
      float distance = delta * distancePerPulse;
      totalDistance[i] += distance;
    }
    float avgDistance = (totalDistance[0] + totalDistance[1] + totalDistance[2] + totalDistance[3]) / 4.0;
    Serial.print("===> Avg Total Distance (m): ");
    Serial.println(avgDistance, 4);
  }
}
