#include <SCServo.h>
SMS_STS st,st2,st3;

// กำหนดพินสำหรับ Cytron 10Amp 5V-30V DC Motor Driver (2 Channels)
const int motor1_DIR = 6;   // dir1
const int motor1_PWM = 7;   // pwm1
const int motor2_DIR = 8;   // dir2
const int motor2_PWM = 9;   // pwm2
const int degree = 90; // องศาการหมุนของ servo
int st_rotate = 0 ; // องศาการหมุน servo หน่อยอนาลอก 0-4095

void setup() {

    //สำหรับการเริ่มต้น  Servo
    Serial.begin(152000);
    Serial2.begin(1000000);
    Serial3.begin(1000000);
    st2.pSerial= &Serial2;
    st3.pSerial = &Serial3;
    while (!Serial2) {} // รอให้ Serial พร้อม
    while (!Serial3) {} // รอให้ Serial พร้อม

    

    // ตั้งค่าพินสำหรับ Cytron 10Amp Motor Driver
    pinMode(motor1_DIR, OUTPUT);  // ทิศทางของมอเตอร์ 1
    pinMode(motor1_PWM, OUTPUT);  // PWM ของมอเตอร์ 1
    pinMode(motor2_DIR, OUTPUT);  // ทิศทางของมอเตอร์ 2
    pinMode(motor2_PWM, OUTPUT);  // PWM ของมอเตอร์ 2

    delay(5000);
}

void loop() {

    // เดินหน้าก่อน
    
    // ขับมอเตอร์ตัวแรกเดินหน้า
    digitalWrite(motor1_DIR, LOW);  // ทิศทางเดินหน้า
    analogWrite(motor1_PWM, 100);    // ความเร็ว 50 (ค่าตั้งแต่ 0 ถึง 255)
    
    // ขับมอเตอร์ตัวที่สองเดินหน้า
    digitalWrite(motor2_DIR, HIGH);  // ทิศทางเดินหน้า
    analogWrite(motor2_PWM, 100);    // ความเร็ว 50
    
    delay(2000);

    //หยุดมอเตอร์
    digitalWrite(motor1_DIR, LOW);  // 
    analogWrite(motor1_PWM, 0);    // 
    
    //หยุดมอเตอร์
    digitalWrite(motor2_DIR, HIGH);  // 
    analogWrite(motor2_PWM, 0);    // 

    delay(1000);


    //หมุน servo 90 องศ่า หมุนด้านซ้าย
    st_rotate = map(degree,0,360,0,4095);

    st2.WritePosEx(1,st_rotate, 1500, 50); // st.WritePosEx(servoID, position, speed, torque);
    st3.WritePosEx(2,st_rotate, 1500, 50);

    delay(5500);
    
    
    //ขับมอเตอร์ถอยหลัง
    
    // เปลี่ยนทิศทางมอเตอร์ตัวแรกเป็นถอยหลัง
    digitalWrite(motor1_DIR, HIGH);   // ทิศทางถอยหลัง
    analogWrite(motor1_PWM, 100);    // ความเร็ว 50
    
    // เปลี่ยนทิศทางมอเตอร์ตัวที่สองเป็นถอยหลัง
    digitalWrite(motor2_DIR, LOW);   // ทิศทางเดินถอยหลัง
    analogWrite(motor2_PWM, 100);    // ความเร็ว 100

    
    delay(2000);

    //หยุดมอเตอร์
    digitalWrite(motor1_DIR, HIGH);  
    analogWrite(motor1_PWM, 0);    
    
    //หยุดมอเตอร์
    digitalWrite(motor2_DIR, LOW);  
    analogWrite(motor2_PWM, 0);    

    
    delay(1000);



    //หมุน servo 90 องศ่า หมุนด้าขวา
    st_rotate = map(degree,0,360,0,4095);

    st2.WritePosEx(1,-st_rotate, 1500, 50); // st.WritePosEx(servoID, position, speed, torque);
    st3.WritePosEx(2,-st_rotate, 1500, 50);

    delay(5500);
    
    
    
}
