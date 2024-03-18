/*
Reciever
*/
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <assert.h>


const byte address[6] = "00001";

RF24 radio(8, 10);  // RF24 is the transmitter/reciever (transciever), 8 & 10 arduino wires to reciever

struct StickValues {
  byte throttle;  // left - up down
  byte yaw;       // left - left right
  byte pitch;     // right - up down
  byte roll;      // right - left right
  byte sw1;
  byte sw2;
};

StickValues data;

void resetData() {
  data.throttle = 64;
  data.yaw = 125;  //middle value
  data.pitch = 144;
  data.roll = 132;
  data.sw1 = 0;
  data.sw2 = 0;
};

void setup() {

  // boiler plate
  resetData();

  Serial.begin(9600);  // ignore

  radio.begin();  // ignore
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);

  radio.startListening();

  // ESC1.attach(9, 1000, 1500); // pin, min pulse width, max pulse width in microseconds
  // ESC1.write(0); // null
  // delay(2000); // sleep mls
  // ESC2.attach(6, 700, 1200); // TODO figure out what these 0's should be (anywhere from 700 to 10000 ?)
  // ESC2.write(0);
  // delay(2000);
  // ESC3.attach(3, 700, 1200);
  // ESC3.write(0);
  // delay(2000);
  // ESC4.attach(5, 700, 1200);
  // ESC4.write(0);
  // delay(2000);
}

void loop() {
  // ! Temp code
  // if (true) {
  //     byte throttle = data.throttle * 180 / 127;
  //     Serial.print(throttle);
  //     Serial.print("\n");

  //     const int val = 100;

  //     ESC1.write(val); // 0 - 180, speed
  //     ESC2.write(val);
  //     ESC3.write(val);
  //     ESC4.write(val);

  //     return;
  // }

  while (radio.available()) {
    radio.read(&data, sizeof(StickValues));  // get into data
    Serial.println(data.throttle);

  }

  // byte throttle = data.throttle * 180 / 127;
  // Serial.print(throttle);
  // Serial.print("\n");

  // const int val = 180;

  // ESC1.write(val);  // 0 - 180, speed
  // ESC2.write(val);
  // ESC3.write(val);
  // ESC4.write(val);
}
