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

SMS_STS st;

#define TIMEOUT_MS 1500 
#define SERVO_DELAY_MS 1000  // ระยะเวลาที่ให้เซอร์โวหมุนก่อน (1 วินาที)
unsigned long lastUpdateTime = 0;
unsigned long servoStartTime = 0;

// โครงสร้างข้อมูลที่รับมาจาก Serial
struct __attribute__((packed)) SwerveData {
    float motor1 = 0;
    float motor2 = 0;
    float motor3 = 0;
    float motor4 = 0;
    float servo1 = 0;
    float servo2 = 0;
    float servo3 = 0;
    float servo4 = 0;
    int32_t checksum = 0;
};

SwerveData receivedData;
SwerveData lastReceivedData;

// สถานะของระบบ
enum SystemState { IDLE, WAITING_SERVO, MOVING_MOTORS };
SystemState currentState = IDLE;

// ฟังก์ชันควบคุมมอเตอร์ MDDA 10A Cytron
void controlMotor(int pwmPin, int dirPin, float speed) {
    int speedint = speed * 10;
    if (speedint > 0) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
        speedint = -speedint;
    }
    analogWrite(pwmPin, speedint);
}

// ฟังก์ชันควบคุม SMS_STS Servo
void controlServo(int id, float angle) {
    int angleint = angle * 100;
    // สมมุติว่า map สำหรับเซอร์โวใช้ช่วง -157 ถึง 157 (สามารถปรับได้ตามการสอบเทียบจริง)
    int pos = map(angleint, -157, 157, 0, 4095);
    int speed = 2500;
    int torque = 50;
    st.WritePosEx(id, pos, speed, torque);
}

// ฟังก์ชันหยุดมอเตอร์ทุกตัว
void stopAllMotors() {
    controlMotor(MOTOR1_PWM, MOTOR1_DIR, 0);
    controlMotor(MOTOR2_PWM, MOTOR2_DIR, 0);
    controlMotor(MOTOR3_PWM, MOTOR3_DIR, 0);
    controlMotor(MOTOR4_PWM, MOTOR4_DIR, 0);
}

void setup() {
    Serial.begin(115200);
    Serial.println("✅ Arduino Mega Ready to Receive Data via Serial...");

    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR3_PWM, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(MOTOR4_PWM, OUTPUT);
    pinMode(MOTOR4_DIR, OUTPUT);

    Serial2.begin(1000000);
    st.pSerial = &Serial2;

    Serial.print("Expected struct size: ");
    Serial.println(sizeof(SwerveData));

    // ตั้งค่าเริ่มต้นให้เซอร์โวอยู่ที่ 90 องศา
    int angleini = 180;
    int degree = map(angleini, 0, 360, 0, 4095);
    st.WritePosEx(1, degree, 1500, 50);
    st.WritePosEx(2, degree, 1500, 50);
    st.WritePosEx(3, degree, 1500, 50);
    st.WritePosEx(4, degree, 1500, 50);
    Serial.println("Servo Set 90 Degree");
    delay(2000);
}

void loop() {
    // 1. อ่านข้อมูลจาก Serial หากมีข้อมูลครบขนาดของ SwerveData
    if (Serial.available() >= sizeof(SwerveData)) {
        Serial.readBytes((uint8_t*)&receivedData, sizeof(SwerveData));
        Serial.println("✅ Data Received Successfully!");

        // แสดงข้อมูลที่ได้รับ
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

        // อัปเดตเวลาล่าสุดที่ได้รับข้อมูล
        lastUpdateTime = millis();
        lastReceivedData = receivedData;

        // ส่งคำสั่งให้เซอร์โวหมุน ถ้ายังไม่ได้อยู่ในสถานะ WAITING_SERVO
        if (currentState != WAITING_SERVO) {
            Serial.println("🔄 Moving Servos First...");
            controlServo(1, receivedData.servo1);
            controlServo(2, receivedData.servo2);
            controlServo(3, receivedData.servo3);
            controlServo(4, receivedData.servo4);

            servoStartTime = millis(); // ตั้งเวลาเริ่มต้นสำหรับการรอเซอร์โว
            currentState = WAITING_SERVO;
        }
    }

    // 2. ตรวจสอบว่าผ่านเวลาที่กำหนดสำหรับเซอร์โวแล้วหรือยัง
    if (currentState == WAITING_SERVO && (millis() - servoStartTime) >= SERVO_DELAY_MS) {
        Serial.println("🚗 Moving Motors...");
        controlMotor(MOTOR1_PWM, MOTOR1_DIR, receivedData.motor1);
        //controlMotor(MOTOR2_PWM, MOTOR2_DIR, receivedData.motor2);
        // กำหนดให้มอเตอร์ 3 และ 4 ทำงานสวนทาง
       // controlMotor(MOTOR3_PWM, MOTOR3_DIR, receivedData.motor3);
        controlMotor(MOTOR4_PWM, MOTOR4_DIR, receivedData.motor4);

        currentState = MOVING_MOTORS;
    }

    // 3. ตรวจสอบ Timeout หากไม่มีข้อมูลใหม่ใน TIMEOUT_MS ให้หยุดมอเตอร์
    if (millis() - lastUpdateTime > TIMEOUT_MS) {
        stopAllMotors();
        currentState = IDLE;
    }
}
