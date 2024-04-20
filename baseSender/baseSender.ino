// #include <Arduino.h>
#define IR_SEND_PIN A2
#include <IRremote.h>
#include <IRremoteInt.h>

// int IrTransmiterPin = A2;

IRsend irsend = IRsend();

void setup()
{  
  Serial.begin(115200);
  // pinMode(IrTransmiterPin, OUTPUT);

}

int counter = 0;

void loop(){
    delay(100);
    irsend.sendSony(2, 2, 2);
}