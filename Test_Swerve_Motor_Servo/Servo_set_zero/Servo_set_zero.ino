#include <SCServo.h>
SMS_STS st; 
int position;
void setup(){
  Serial2.flush();
  Serial2.begin(1000000);
  st.pSerial=&Serial2;
  while (!Serial2) {}
}
void loop(){
  st.WritePosEx(3,0, 1500, 50);
  //st.WritePosEx(2,0, 1500, 50);
  //st.WritePosEx(1,0, 1500, 50);
  //st.WritePosEx(4,0, 1500, 50);
  //st.WritePosEx(1,0, 1500, 50);
delay(3000);                     // Wait for 1 second (adjust based on your servo's speed) 
}
