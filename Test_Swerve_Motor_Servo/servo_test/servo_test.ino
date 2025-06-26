#include <SCServo.h>
SMS_STS st; 
int position;
void setup(){
  Serial2.flush();
  Serial2.begin(1000000);
  st.pSerial=&Serial2; //Serial1 คือ Tx1 นะครับ น่าจะช่อง 18
  while (!Serial2) {}
}
void loop(){
  //st.WritePosEx(1,4095, 1500, 50); // st.WritePosEx(servoID, position, speed, torque);
  st.WritePosEx(1,4095, 1500, 50);
delay(4000);                     // Wait for 5 second (adjust based on your servo's speed)
//st.WritePosEx(1, 0, 1500, 50);
st.WritePosEx(1, 0, 1500, 50);
delay(4000); 
}

// position 0 องศา = 0
//360 องศา = 4096

  /*
st.WritePosEx(1,4095, 1500, 50);
delay(4000);                     // Wait for 1 second (adjust based on your servo's speed)
st.WritePosEx(1,-4095, 1500, 50);
delay(4000); 
   */ 
