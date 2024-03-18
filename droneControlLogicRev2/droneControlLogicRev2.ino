//int xMap, yMap, xValue, yValue;

#define VRx1 A0 //variable resistance 
#define VRx2 A2
#define VRy1 A1
#define VRy2 A3
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>


const byte address[6] = "00001";
RF24 radio(10,8);

struct MyValues{
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte sw1; 
  byte sw2;
};

MyValues data;

void resetData(){
  data.throttle = 64;
  data.yaw = 125; //middle value 
  data.pitch = 144;
  data.roll = 132;
  data.sw1 = 0;
  data.sw2 = 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(3, INPUT);
  pinMode(6, INPUT);
  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(address);
  radio.stopListening();
  resetData();
}

int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}
 
void loop() {
  data.throttle = mapJoystickValues(analogRead(VRy1), 13, 0 , 1015, true);
  data.yaw = mapJoystickValues(analogRead(VRx1), 1, 505, 1020, true);
  data.pitch = mapJoystickValues(analogRead(VRy2), 12, 544, 1021, true);
  data.roll = mapJoystickValues(analogRead(VRx2),34, 522, 1020, true);
  data.sw1 = digitalRead(3);
  data.sw2 = digitalRead(6);
  //xValue1 = analogRead(VRx1);
  //yValue1 = analogRead(VRy1);
  //xMap = map(xValue, 0,1023, 0, 7);
  //yMap = map(yValue,0,1023,7,0);
  Serial.print("throttle ");
  Serial.println(data.throttle);
  Serial.print("yaw ");
  Serial.println(data.yaw);
  Serial.print("pitch ");
  Serial.println(data.pitch);
  Serial.print("roll ");
  Serial.println(data.roll);
  Serial.print("button 1 ");
  Serial.println(data.sw1);
  Serial.print("button 2 ");
  Serial.println(data.sw2);
  Serial.print(radio.available());
  delay(50);
  radio.write(&data, sizeof(data));
}
