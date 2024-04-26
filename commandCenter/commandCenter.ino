#include <Arduino.h>
#define IR_SEND_PIN A2
#include <IRremote.h>
#include <IRremoteInt.h>

IRsend irsend = IRsend();

void setup()
{  
  Serial.begin(115200);
  // pinMode(IrTransmiterPin, OUTPUT);

}

int counter = 0;

void loop(){
  delayMicroseconds(500);
  // IR sender, just send a code
  irsend.sendSony(2, 2, 2);
}