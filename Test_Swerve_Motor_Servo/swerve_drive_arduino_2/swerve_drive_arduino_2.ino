#include <Arduino.h>
#include <SMS_STS.h>

#define BAUD_RATE 115200

// พินควบคุม MDDA 10A Cytron (PWM + Direction)
#define MOTOR1_PWM 4
#define MOTOR1_DIR 5
#define MOTOR2_PWM 6
#define MOTOR2_DIR 7
#define MOTOR3_PWM 8
#define MOTOR3_DIR 9
#define MOTOR4_PWM 10
#define MOTOR4_DIR 11

// พินสำหรับ Serial ของ SMS_STS (ใช้ Serial2)
//#define SERVO_TX 16
//#define SERVO_RX 3

SMS_STS st;  // Object สำหรับ SMS_STS Servo

// Timeout ตั้งไว้ที่ 1 วินาที
#define TIMEOUT_MS 1000
unsigned long lastUpdateTime = 0;

// โครงสร้างข้อมูล
struct __attribute__((packed)) SwerveData {
    float motor1 = 0;
    float motor2 = 0;
    float motor3 = 0;
    float motor4 = 0;
    float servo1 = 0;  // เริ่มที่ 90 องศา
    float servo2 = 0;
    float servo3 = 0;
    float servo4 = 0;
    int32_t checksum = 0; 
};

// ตัวแปรเก็บข้อมูล
SwerveData receivedData;
SwerveData lastReceivedData; // ย้ายการประกาศมาไว้ที่นี่

// ฟังก์ชันควบคุม MDDA 10A Cytron
void controlMotor(int pwmPin, int dirPin, float speed) {
    int speedint = speed * 10 ;
    
    if (speedint > 0) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
        speedint = -speedint;  // เปลี่ยนค่าเป็นบวกสำหรับ PWM
    }
    Serial.println(speedint);
    analogWrite(pwmPin, speedint);
}
  
// ฟังก์ชันควบคุม SMS_STS Servo
void controlServo(int id, float angle) {
    int angleint = angle *100;
    int pos = map(angleint, -157, 157, 0, 4095);  // SMS_STS รองรับ 0-360 องศา (0-4095)
    //Serial.println(pos);
    int speed = 2500;  // กำหนดความเร็ว
    int torque = 50;   // กำหนดแรงบิด
    st.WritePosEx(id, pos, speed, torque);
}

void stopAllMotors() {
    controlMotor(MOTOR1_PWM, MOTOR1_DIR, 0);
    controlMotor(MOTOR2_PWM, MOTOR2_DIR, 0);
    controlMotor(MOTOR3_PWM, MOTOR3_DIR, 0);
    controlMotor(MOTOR4_PWM, MOTOR4_DIR, 0);
    //Serial.println("⏹ Motors Stopped (Timeout)");
}

void setup() {
    Serial.begin(115200);  // Serial สำหรับการสื่อสารกับ Python
    Serial.println("✅ Arduino Mega Ready to Receive Data via Serial...");

    // ตั้งค่ามอเตอร์ MDDA 10A Cytron
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR3_PWM, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(MOTOR4_PWM, OUTPUT);
    pinMode(MOTOR4_DIR, OUTPUT);

    // ตั้งค่า SMS_STS Servo
    Serial2.begin(1000000);  // ใช้ Serial1 ที่ 1Mbps สำหรับ SMS_STS
    st.pSerial = &Serial2;  

    Serial.print("Expected struct size: ");
    Serial.println(sizeof(SwerveData));

    
    int angleini = 180,degree;
    degree = map(angleini, 0, 360, 0, 4095);
    
   
   /* // ตั้งค่าเริ่มต้นให้เซอร์โวอยู่ที่ 90 องศา
    st.WritePosEx(1, degree, 1500, 50);
    st.WritePosEx(2, degree, 1500, 50);
    st.WritePosEx(3, degree, 1500, 50);
    st.WritePosEx(4, degree, 1500, 50);
    Serial.print("Servo Set 90 Degree");
    delay(2000);*/
}

void loop() {
   if (Serial.available() >= sizeof(SwerveData)) {
        Serial.readBytes((uint8_t*)&receivedData, sizeof(SwerveData));

        Serial.println("✅ Data Received Successfully!");

        // แสดงค่าที่ได้รับจาก Python
        Serial.print("Motor Values: ");
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

        // ตรวจสอบความถูกต้องของข้อมูล
        /*if (receivedData.checksum != (int32_t)(receivedData.motor1 + receivedData.motor2 + 
                                               receivedData.motor3 + receivedData.motor4 + 
                                               receivedData.servo1 + receivedData.servo2 + 
                                               receivedData.servo3 + receivedData.servo4)) {
            Serial.println("⚠️ Mismatch Data! Ignoring...");
            return;
        }*/

        // ตรวจสอบว่าข้อมูลเปลี่ยนแปลงหรือไม่
        /*if (memcmp(&receivedData, &lastReceivedData, sizeof(SwerveData)) == 0) {
            if (millis() - lastUpdateTime > TIMEOUT_MS) {
                stopAllMotors();
            }
            return;
        }*/

        // อัปเดตเวลาล่าสุด
        lastUpdateTime = millis();
        lastReceivedData = receivedData; // เก็บค่าใหม่

        // หมุนเซอร์โวก่อนมอเตอร์เคลื่อนที่
        //Serial.println("🔄 Moving Servos First...");
        //controlServo(1, receivedData.servo1);
        //controlServo(2, receivedData.servo2);
        //controlServo(3, receivedData.servo3);
        //controlServo(4, receivedData.servo4);
        //delay(1000);  // หน่วงเวลาให้เซอร์โวขยับก่อน

        // ควบคุมมอเตอร์
        Serial.println("🚗 Moving Motors...");
        controlMotor(MOTOR1_PWM, MOTOR1_DIR, receivedData.motor1);
        //controlMotor(MOTOR2_PWM, MOTOR2_DIR, receivedData.motor2);
        //controlMotor(MOTOR3_PWM, MOTOR3_DIR, -receivedData.motor3); // มอเตอร์ 3 ตรงข้ามมอเตอร์ 1
        controlMotor(MOTOR4_PWM, MOTOR4_DIR, receivedData.motor4); // มอเตอร์ 4 ตรงข้ามมอเตอร์ 2*/

        Serial.println("------------------------------");
    }

    // ตรวจสอบ timeout หยุดมอเตอร์หากไม่มีข้อมูลใหม่ใน 1 วินาที
    /*if (millis() - lastUpdateTime > TIMEOUT_MS) {
        stopAllMotors();
    }*/
}
