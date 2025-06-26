// ✅ Combined Swerve Drive Control with Encoder PID (IMU Temporarily Disabled)
#include <Arduino.h>
#include <SMS_STS.h>
#include <PinChangeInterrupt.h>

#define BAUD_RATE 115200

// === Cytron MDDA10 Motor Pins ===
#define MOTOR1_PWM 4
#define MOTOR1_DIR 5
#define MOTOR2_PWM 6
#define MOTOR2_DIR 7
#define MOTOR3_PWM 8
#define MOTOR3_DIR 9
#define MOTOR4_PWM 10
#define MOTOR4_DIR 11

// === Encoder Pins ===
const uint8_t encoderPinsA[4] = {A8, A10, A12, A14};
const uint8_t encoderPinsB[4] = {A9, A11, A13, A15};

// === Timeout Settings ===
#define COMMAND_TIMEOUT_MS 9000
#define SAFETY_TIMEOUT_MS 18000
#define SERVO_DELAY_MS 1000

// === Encoder and PID ===
volatile long encoderCount[4] = {0};
long prevEncoderCount[4] = {0};
float currentRPM[4] = {0};
float targetRPM[4] = {0};
float Kp = 1.2, Ki = 0.15, Kd = 0.05;
float integral[4] = {0};
float prevError[4] = {0}; 
const int pulsesPerRev = 360;
const float wheelRadius = 0.12;

// === Swerve Protocol ===
SMS_STS st;
struct __attribute__((packed)) SwerveData {
  float motor1, motor2, motor3, motor4;
  float servo1, servo2, servo3, servo4;
  int8_t rotate_direction;
  float rotate_angle;
  int32_t checksum;
};
SwerveData receivedData;
SwerveData nextData;
bool hasNewCommand = false;
bool isTimedOut = false;
bool firstTimeoutOccurred = false;

unsigned long lastUpdateTime = 0;
unsigned long servoStartTime = 0;
unsigned long lastControlTime = 0;

enum SystemState { IDLE, WAITING_SERVO, MOVING_MOTORS };
SystemState currentState = IDLE;

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

void stopAllMotors() {
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
  analogWrite(MOTOR3_PWM, 0);
  analogWrite(MOTOR4_PWM, 0);
}

void controlServo(int id, float angle) {
  int angleint = angle * 100;
  int pos = map(angleint, -157, 157, 0, 4095);
  st.WritePosEx(id, pos, 2500, 150);
}

void updatePIDControl() {
  unsigned long now = millis();
  if (now - lastControlTime < 33) return;
  float dt = (now - lastControlTime) / 1000.0;
  lastControlTime = now;

  for (int i = 0; i < 4; i++) {
    long delta = encoderCount[i] - prevEncoderCount[i];
    prevEncoderCount[i] = encoderCount[i];
    currentRPM[i] = (delta * 60.0) / (pulsesPerRev * dt);
    float speed_mps = delta * (2 * PI * wheelRadius / pulsesPerRev) / dt;

    float error = targetRPM[i] - currentRPM[i];
    integral[i] += error * dt;
    float derivative = (error - prevError[i]) / dt;
    prevError[i] = error;

    float output = Kp * error + Ki * integral[i] + Kd * derivative;
    int pwm = constrain(abs(output), 0, 150);
    if (pwm > 0 && pwm < 100) pwm = 100; // ✅ Added minimum threshold fix to overcome static friction
    analogWrite(MOTOR1_PWM + i * 2, pwm);
    digitalWrite(MOTOR1_DIR + i * 2, targetRPM[i] >= 0 ? HIGH : LOW);

    Serial.print("Motor "); Serial.print(i + 1);
    Serial.print(" | Target: "); Serial.print(targetRPM[i]);
    Serial.print(" | SimPWM: "); Serial.println(targetRPM[i] * 15);
  }

  Serial.print("Encoder Raw: ");
  for (int i = 0; i < 4; i++) {
    Serial.print("M"); Serial.print(i + 1); Serial.print(": ");
    Serial.print(encoderCount[i]); Serial.print("\t");
  }
  Serial.println();
  Serial.println("--------------------------------------------------");
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial2.begin(1000000);
  st.pSerial = &Serial2;
  setupEncoders();

  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR1_PWM + i * 2, OUTPUT);
    pinMode(MOTOR1_DIR + i * 2, OUTPUT);
  }

  Serial.println("✅ Arduino Mega Ready with Encoder (IMU Disabled)");
  int degree = map(180, 0, 360, 0, 4095);
  for (int i = 1; i <= 4; i++) controlServo(i, 0);
  delay(2000);
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

    if (currentState == IDLE) {
      receivedData = nextData;
      hasNewCommand = false;
      servoStartTime = millis();
      currentState = WAITING_SERVO;

      // 🔄 Reset PID memory when a new command is issued
      for (int i = 0; i < 4; i++) {
        integral[i] = 0;
        prevError[i] = 0;
      }
    } else {
      hasNewCommand = true;
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
      for (int i = 0; i < 4; i++) targetRPM[i] = 0;
      if (millis() - servoStartTime <= SERVO_DELAY_MS) {
        controlServo(1, receivedData.servo1);
        controlServo(2, receivedData.servo2);
        controlServo(3, receivedData.servo3);
        controlServo(4, receivedData.servo4);
      } else {
        currentState = MOVING_MOTORS;
      }
      break;

    case MOVING_MOTORS:
      targetRPM[0] = receivedData.motor1 * 15;
      targetRPM[1] = receivedData.motor2 * 15;
      targetRPM[2] = -receivedData.motor3 * 15;
      targetRPM[3] = -receivedData.motor4 * 15;
      updatePIDControl();
      if (hasNewCommand) {
        receivedData = nextData;
        hasNewCommand = false;
        servoStartTime = millis();
        currentState = WAITING_SERVO;
      } else {
        currentState = IDLE;
      }
      break;

    case IDLE:
      for (int i = 0; i < 4; i++) targetRPM[i] = 0;
      break;
  }
}
