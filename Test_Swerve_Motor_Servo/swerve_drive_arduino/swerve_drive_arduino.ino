#include <Arduino.h>

#define BAUD_RATE 115200
#define TIMEOUT_MS 1000  // 1 วินาที (1000 มิลลิวินาที)

// โครงสร้างข้อมูลที่รับจาก ROS2
struct SwerveData {
    float motor1;
    float motor2;
    float motor3;
    float motor4;
    float servo1;
    float servo2;
    float servo3;
    float servo4;
    uint8_t checksum;
};

// ตัวแปรเก็บข้อมูล
SwerveData receivedData;

// เวลาล่าสุดที่ได้รับข้อมูล
unsigned long lastReceivedTime = 0;

// ฟังก์ชันคำนวณ checksum (XOR)
uint8_t calculateChecksum(uint8_t *data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void setup() {
    Serial.begin(BAUD_RATE);
    Serial.println("✅ Arduino Mega Ready to Receive Data...");
    lastReceivedTime = millis();  // ตั้งค่าเริ่มต้นของเวลา
}

void loop() {
    if (Serial.available() >= sizeof(SwerveData)) {
        // อ่านข้อมูลจาก Serial ลงใน struct
        Serial.readBytes((uint8_t*)&receivedData, sizeof(SwerveData));

        // คำนวณ checksum ใหม่
        uint8_t calculatedChecksum = calculateChecksum((uint8_t*)&receivedData, sizeof(SwerveData) - 1);

        if (calculatedChecksum == receivedData.checksum) {
            // ✅ ข้อมูลถูกต้อง อัปเดตเวลาล่าสุด
            lastReceivedTime = millis();

            // แสดงค่าที่ได้รับ
            Serial.println("✅ Data Received Successfully!");
            Serial.print("Motor Speeds: ");
            Serial.print(receivedData.motor1); Serial.print(", ");
            Serial.print(receivedData.motor2); Serial.print(", ");
            Serial.print(receivedData.motor3); Serial.print(", ");
            Serial.println(receivedData.motor4);

            Serial.print("Servo Angles: ");
            Serial.print(receivedData.servo1); Serial.print(", ");
            Serial.print(receivedData.servo2); Serial.print(", ");
            Serial.print(receivedData.servo3); Serial.print(", ");
            Serial.println(receivedData.servo4);

            Serial.println("------------------------------");
        } else {
            // ❌ แจ้งเตือนหาก Checksum ไม่ตรง
            Serial.println("❌ Checksum Mismatch! Data Corrupted.");
        }
    }

    // ตรวจสอบ Timeout (ถ้าไม่มีข้อมูลเข้ามาเกิน 1 วินาที)
    if (millis() - lastReceivedTime > TIMEOUT_MS) {
        receivedData.motor1 = 0;
        receivedData.motor2 = 0;
        receivedData.motor3 = 0;
        receivedData.motor4 = 0;

        Serial.println("⏳ Timeout! Stopping motors, keeping last servo position.");
        Serial.print("Motor Speeds: ");
        Serial.print(receivedData.motor1); Serial.print(", ");
        Serial.print(receivedData.motor2); Serial.print(", ");
        Serial.print(receivedData.motor3); Serial.print(", ");
        Serial.println(receivedData.motor4);

        Serial.print("Servo Angles (unchanged): ");
        Serial.print(receivedData.servo1); Serial.print(", ");
        Serial.print(receivedData.servo2); Serial.print(", ");
        Serial.print(receivedData.servo3); Serial.print(", ");
        Serial.println(receivedData.servo4);

        Serial.println("------------------------------");
        lastReceivedTime = millis();  // ป้องกันการพิมพ์ซ้ำบ่อยเกินไป
    }
}
