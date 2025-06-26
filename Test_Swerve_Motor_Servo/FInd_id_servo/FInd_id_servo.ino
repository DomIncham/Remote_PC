#include <SCServo.h>
SMS_STS st;

void setup() {
  Serial.begin(115200); // เปิด Serial Monitor เพื่อดูผลลัพธ์
  Serial2.begin(1000000); // เปิด Serial1 สำหรับสื่อสารกับเซอร์โว
  st.pSerial = &Serial2; // กำหนดให้ไลบรารีใช้ Serial1
  while (!Serial2) {} // รอให้ Serial1 พร้อม
}

void loop() {
  uint8_t id = 1; // เริ่มตรวจสอบจาก ID 1
  bool servoFound = false;

  while (id <= 253) { // ตรวจสอบ ID ตั้งแต่ 1 ถึง 253
    if (st.Ping(id) == true) { // ถ้าเจอเซอร์โวที่มี ID นี้
      Serial.print("Found servo with ID: ");
      Serial.println(id);
      servoFound = true;
      break; // หยุดการค้นหาหากเจอเซอร์โว
    }
    id++; // ตรวจสอบ ID ถัดไป
  }

  if (!servoFound) {
    Serial.println("No servo found!");
  }

  delay(1000); // หน่วงเวลา 5 วินาทีก่อนตรวจสอบอีกครั้ง
}
