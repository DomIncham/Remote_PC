#include <Arduino.h>

#define BAUD_RATE 115200

// กำหนดโครงสร้างแบบ packed เพื่อลดปัญหา memory alignment
struct __attribute__((packed)) SwerveData {
    float motor1=0;
    float motor2=0;
    float motor3=0;
    float motor4=0;
    float servo1=0;
    float servo2=0;
    float servo3=0;
    float servo4=0;
    int32_t checksum=0; // Checksum เป็น int ตามที่ส่งมาจาก Python
};

// ตัวแปรเก็บข้อมูล
SwerveData receivedData;

void setup() {
    Serial.begin(115200);  // ใช้ Serial ปกติสำหรับรับข้อมูลจาก Python
    Serial.println("✅ Arduino Mega Ready to Receive Data via Serial...");
    Serial.print("Expected struct size: ");
    Serial.println(sizeof(SwerveData));  // แสดงขนาดของโครงสร้าง
}

void loop() {
    if (Serial.available() >= sizeof(SwerveData)) {
        // อ่านข้อมูลจาก Serial ลงใน struct
        Serial.readBytes((uint8_t*)&receivedData, sizeof(SwerveData));

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

        Serial.print("Checksum: ");
        Serial.println(receivedData.checksum);

        Serial.println("------------------------------");
    }
}
