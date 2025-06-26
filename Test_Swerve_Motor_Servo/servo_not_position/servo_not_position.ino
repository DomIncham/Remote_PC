#include <SCServo.h>

SMS_STS st;

void setup() {
  Serial.begin(115200);    // Serial Monitor สำหรับ debug
  Serial1.begin(1000000);  // ใช้ Serial1 สื่อสารกับเซอร์โว

  st.pSerial = &Serial1;

  Serial.println("Testing servo in Speed Mode...");
}

void loop() {
  // ส่งคำสั่งควบคุมในโหมดความเร็ว
  st.WriteSpe(1, 200, 50); // ID=1, Speed=200 (หมุนตามเข็มนาฬิกา)
  delay(2000);

  st.WriteSpe(1, -200, 50); // ID=1, Speed=-200 (หมุนทวนเข็มนาฬิกา)
  delay(2000);
}
