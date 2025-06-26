#include <SCServo.h>

SMS_STS st;

int ID_ChangeFrom = 3;   // The original ID of the servo whose ID is to be changed is factory defaulted to 1.
int ID_Changeto = 1;     // New ID

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000);  // เริ่มต้นการเชื่อมต่อ Serial1 ที่ Baud rate 1Mbps
  st.pSerial = &Serial2;
  while (!Serial2) {}  // รอการเชื่อมต่อกับ Serial1

  // ปลดล็อก EPROM เพื่ออนุญาตให้เปลี่ยนแปลงข้อมูลภายใน SCServo
  st.unLockEprom(ID_ChangeFrom);

  // เปลี่ยน ID ของ Servo จาก ID_ChangeFrom ไปยัง ID_Changeto
  st.writeByte(ID_ChangeFrom, SMS_STS_ID, ID_Changeto);  // SMS_STS_ID เป็นค่า Constant สำหรับการเปลี่ยน ID

  // ล็อค EPROM หลังจากเปลี่ยน ID
  st.LockEprom(ID_Changeto);

  // แสดงผลการเปลี่ยน ID ใน Serial Monitor
  Serial.println("✅ ID has been successfully changed.");
}

void loop() {
  // ไม่มีการทำงานอื่นใน loop, จึงไม่ต้องการรหัสในส่วนนี้
}
