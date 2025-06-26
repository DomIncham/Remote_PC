  #include <Arduino.h>
  #include <SMS_STS.h>
  
  #define BAUD_RATE 115200
  
  // Cytron MDDA10 Motor pins
  #define MOTOR1_PWM 4
  #define MOTOR1_DIR 5
  #define MOTOR2_PWM 6
  #define MOTOR2_DIR 7
  #define MOTOR3_PWM 8
  #define MOTOR3_DIR 9
  #define MOTOR4_PWM 10
  #define MOTOR4_DIR 11
  
  #define TIMEOUT_MS 1500
  #define SERVO_DELAY_MS 500  // wait after moving servo before motors
  
  SMS_STS st,st2;
  
  // Struct for receiving data
  struct __attribute__((packed)) SwerveData {
    float motor1, motor2, motor3, motor4;
    float servo1, servo2, servo3, servo4;
    int32_t checksum;   
  };
  
  SwerveData receivedData;
  SwerveData nextData;
  bool hasNewCommand = false;
  bool isTimedOut = false;
  
  unsigned long lastUpdateTime = 0;
  unsigned long servoStartTime = 0;
  
  bool newDataReceived = false;
  
  enum SystemState { IDLE, WAITING_SERVO, MOVING_MOTORS };
  SystemState currentState = IDLE;
  
  void controlMotor(int pwmPin, int dirPin, float speed) {
    //int speedint = speed * 11;
    int speedint = map(speed,-10.7,10.7,-130,130);
    if (speedint > 0) {
      digitalWrite(dirPin, HIGH);
    } else {
      digitalWrite(dirPin, LOW);
      speedint = -speedint;
    }
    Serial.print("Speed Command: ");
    Serial.println(speedint);
    analogWrite(pwmPin, speedint);
  }
  
  void controlServo(int id, float angle, int num) {
    int angleint = angle * 100;
    int pos = map(angleint, -157, 157, 0, 4095);
    int speed = 2000;
    int torque = 50;
    if(num == 1){st.WritePosEx(id, pos, speed, torque);}
    if(num == 2){st2.WritePosEx(id, pos, speed, torque);}
    
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
    Serial3.begin(1000000);
    st2.pSerial = &Serial3;
  
    Serial.println("✅ Arduino Mega Ready to Receive Data");
    //Serial.print("Expected struct size: ");
    //Serial.println(sizeof(SwerveData));
  
    pinMode(MOTOR1_PWM, OUTPUT); pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT); pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR3_PWM, OUTPUT); pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(MOTOR4_PWM, OUTPUT); pinMode(MOTOR4_DIR, OUTPUT);
  
        // ตั้งค่าเริ่มต้นให้เซอร์โวอยู่ที่ 90 องศา
      int angleini = 180;
      int degree = map(angleini, 0, 360, 0, 4095);
      st.WritePosEx(1, degree, 1500, 50);
      st.WritePosEx(2, degree, 1500, 50);
      st2.WritePosEx(1, degree, 1500, 50);
      st2.WritePosEx(2, degree, 1500, 50);
      Serial.println("Servo Set 90 Degree");
      delay(2000);
  
    stopAllMotors();
  }

 
void loop() {
  // 0. ตรวจว่า Serial buffer overflow หรือไม่ (เพื่อความปลอดภัย)
  if (Serial.available() > 100) {
    Serial.println("⚠️ Overflow! Clearing Serial buffer...");
    while (Serial.available()) Serial.read();
    currentState = IDLE;
    return;
  }

  // 1. รับข้อมูลจาก ROS2 → ลงใน nextData
  if (Serial.available() >= sizeof(SwerveData)) {
    Serial.readBytes((uint8_t*)&nextData, sizeof(SwerveData));
    while (Serial.available()) Serial.read();  // ล้าง buffer ที่ค้าง
    lastUpdateTime = millis();  // อัปเดตเวลาล่าสุดที่รับข้อมูล
    isTimedOut = false;  // ✅ มีคำสั่งใหม่แล้ว → ออกจากสถานะ timeout

      // DEBUG: ดูค่าทุกครั้งที่รับข้อมูลใหม่
    Serial.println("✅ Data Received!");
    Serial.print("Motors: ");
    Serial.print(receivedData.motor1); Serial.print(", ");
    Serial.print(receivedData.motor2); Serial.print(", ");
    Serial.print(receivedData.motor3); Serial.print(", ");
    Serial.println(receivedData.motor4);

    Serial.print("Servos: ");
    Serial.print(receivedData.servo1); Serial.print(", ");
    Serial.print(receivedData.servo2); Serial.print(", ");
    Serial.print(receivedData.servo3); Serial.print(", ");
    Serial.println(receivedData.servo4);

    Serial.print("Checksum: ");
    Serial.println(receivedData.checksum);
    Serial.println("------------------------------");

    if (currentState == IDLE) {
      // ถ้าหุ่นอยู่ในสถานะว่าง → ใช้คำสั่งใหม่ทันที
      receivedData = nextData;
      hasNewCommand = false;
      servoStartTime = millis();
      currentState = WAITING_SERVO;
      Serial.println("⏱️ New command started immediately");
    } else {
      // ถ้ายังทำงานอยู่ → เก็บคำสั่งไว้ใช้รอบถัดไป
      hasNewCommand = true;
      Serial.println("🕒 New command queued for next execution");
    }
  }

  // 2. รอเวลาหน่วงให้ servo หมุนก่อน (delay servo)
  if (currentState == WAITING_SERVO && millis() - servoStartTime >= SERVO_DELAY_MS) {
    Serial.println("🔄 Moving Servos...");
    controlServo(1, receivedData.servo1, 1); // servo ด้านหน้า - ซ้าย
    controlServo(2, receivedData.servo2, 1); // servo ด้านหน้า - ขวา
    controlServo(1, receivedData.servo3, 2); // servo ด้านหลัง - ซ้าย
    controlServo(2, receivedData.servo4, 2); // servo ด้านหลัง - ขวา
    currentState = MOVING_MOTORS;
  }

  // 3. หลัง servo หมุนเสร็จ → ขยับล้อ
  if (currentState == MOVING_MOTORS) {
    Serial.println("🚗 Moving Motors...");
    controlMotor(MOTOR1_PWM, MOTOR1_DIR, receivedData.motor1);// ล้อสวน
    controlMotor(MOTOR2_PWM, MOTOR2_DIR, -receivedData.motor2); 
    //controlMotor(MOTOR3_PWM, MOTOR3_DIR, receivedData.motor3);  
    //controlMotor(MOTOR4_PWM, MOTOR4_DIR, -receivedData.motor4);  // ล้อสวน

    if (hasNewCommand) {
      // ถ้ามีคำสั่งใหม่รออยู่ → ใช้คำสั่งนั้นทันที (แบบต่อเนื่อง)
      receivedData = nextData;
      hasNewCommand = false;
      servoStartTime = millis();
      currentState = WAITING_SERVO;
      Serial.println("⚡ Executing queued command");
    } else {
      // ถ้าไม่มีคำสั่งใหม่ → กลับไปว่าง
      currentState = IDLE;
      Serial.println("✅ Done. No queued command.");
    }
  }

  // 4. ถ้าไม่มีคำสั่งใหม่มานานเกิน TIMEOUT_MS → หยุดล้อ
  if (!isTimedOut && millis() - lastUpdateTime > TIMEOUT_MS) {
    stopAllMotors();
    currentState = IDLE;
    isTimedOut = true;  // ✅ บอกว่า timeout แล้ว
    Serial.println("🛑 Timeout! Motors stopped.");
  }
}



  
