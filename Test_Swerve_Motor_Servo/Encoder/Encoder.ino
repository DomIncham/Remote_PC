#include <Arduino.h>
#include <PinChangeInterrupt.h>

// === Encoder Pins ===
const uint8_t encoderPinsA[4] = {A8, A10, A12, A14};
const uint8_t encoderPinsB[4] = {A9, A11, A13, A15};

// === Encoder Data ===
volatile long encoderCount[4] = {0};
long prevEncoderCount[4] = {0};
float currentRPM[4] = {0};
float totalDistance[4] = {0}; // ระยะทางสะสมของแต่ละล้อ

// === Wheel Parameters ===
const int pulsesPerRev = 360;
const float wheelRadius = 0.12; // meters
const float distancePerPulse = 2 * PI * wheelRadius / pulsesPerRev;

// === Timing ===
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // milliseconds

// === Encoder ISRs ===
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

void setup() {
  Serial.begin(115200);
  setupEncoders();
  Serial.println("✅ Encoder-only Mode Started");
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrintTime >= printInterval) {
    lastPrintTime = now;
    float dt = printInterval / 1000.0;

    Serial.println("==== Encoder Readings ====");

    float totalAvgDistance = 0;

    for (int i = 0; i < 4; i++) {
      long delta = encoderCount[i] - prevEncoderCount[i];
      prevEncoderCount[i] = encoderCount[i];

      currentRPM[i] = (delta * 60.0) / (pulsesPerRev * dt);
      float distance = delta * distancePerPulse;
      totalDistance[i] += distance;
      totalAvgDistance += totalDistance[i];

      Serial.print("Wheel "); Serial.print(i + 1);
      Serial.print(" | Δpulse: "); Serial.print(delta);
      Serial.print(" | RPM: "); Serial.print(currentRPM[i], 2);
      Serial.print(" | Distance (m): "); Serial.println(totalDistance[i], 4);
    }

    Serial.print("===> Avg Total Distance (m): ");
    Serial.println(totalAvgDistance / 4.0, 4);
    Serial.println("-------------------------------");
  }
}
