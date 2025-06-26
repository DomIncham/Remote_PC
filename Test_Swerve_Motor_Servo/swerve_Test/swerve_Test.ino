#include <SCServo.h>
SMS_STS st;

// กำหนดพินสำหรับ Cytron 10Amp 5V-30V DC Motor Driver (2 Channels)
const int motor1_PWM = 8;   // pwm1
const int motor1_DIR = 9;   // dir1
const int motor2_PWM = 8;   // pwm2
const int motor2_DIR = 9 ;   // dir2
/*const int motor3_PWM = 8;   // pwm3
const int motor3_DIR = 9;   // dir3
const int motor4_PWM = 10;   // pwm4
const int motor4_DIR = 11;   // dir4*/


void setup() {
    Serial.begin(152000);
    //Serial1.begin(1000000);
    //st.pSerial = &Serial1;
    //while (!Serial) {} // รอให้ Serial พร้อม

    // ตั้งค่าพินสำหรับ Cytron 10Amp Motor Driver
    /*pinMode(motor1_DIR, OUTPUT);  // ทิศทางของมอเตอร์ 1
    pinMode(motor1_PWM, OUTPUT);  // PWM ของมอเตอร์ 1*/
    pinMode(motor2_DIR, OUTPUT);  // ทิศทางของมอเตอร์ 2
    pinMode(motor2_PWM, OUTPUT);  // PWM ของมอเตอร์ 2
    /*pinMode(motor3_DIR, OUTPUT);  // ทิศทางของมอเตอร์ 1
    pinMode(motor3_PWM, OUTPUT);  // PWM ของมอเตอร์ 1
    pinMode(motor4_DIR, OUTPUT);  // ทิศทางของมอเตอร์ 2
    pinMode(motor4_PWM, OUTPUT);  // PWM ของมอเตอร์ 2*/

    delay(5000);
}

void loop() {
    // ขับมอเตอร์ตัวแรกเดินหน้า
    /*digitalWrite(motor1_DIR, LOW);  // ทิศทางเดินหน้า
    analogWrite(motor1_PWM, 130);    // ความเร็ว 50 (ค่าตั้งแต่ 0 ถึง 255)*/
    
    // ขับมอเตอร์ตัวที่สองเดินหน้า
    digitalWrite(motor2_DIR, LOW);  // ทิศทางเดินหน้า
    analogWrite(motor2_PWM, 110);    // ความเร็ว 50
      
    delay(2000);

    //หยุดมอเตอร์
    /*digitalWrite(motor1_DIR, LOW);  // 
    analogWrite(motor1_PWM, 0);    // */
    
    //หยุดมอเตอร์
    digitalWrite(motor2_DIR, LOW);  // 
    analogWrite(motor2_PWM, 0);    // 

    delay(1000);

    
   
    
    // เปลี่ยนทิศทางมอเตอร์ตัวแรกเป็นถอยหลัง
    /*digitalWrite(motor1_DIR, HIGH);   // ทิศทางถอยหลัง
    analogWrite(motor1_PWM, 130);    // ความเร็ว 50*/
    
    // เปลี่ยนทิศทางมอเตอร์ตัวที่สองเป็นถอยหลัง
    digitalWrite(motor2_DIR, HIGH);   // ทิศทางเดินถอยหลัง
    analogWrite(motor2_PWM, 110);    // ความเร็ว 100

    
    delay(2000);

    //หยุดมอเตอร์
    /*digitalWrite(motor1_DIR, HIGH);  
    analogWrite(motor1_PWM, 0);   */ 
    
    //หยุดมอเตอร์
    digitalWrite(motor2_DIR, HIGH);  
    analogWrite(motor2_PWM, 0);    

    
    delay(1000);
    
    
}
