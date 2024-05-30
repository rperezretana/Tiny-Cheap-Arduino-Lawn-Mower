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
  // IR sender, just send a code
  int counter = 100;
  Serial.println("Start Signals");
  while(counter>0)
  {
    irsend.sendSony(2, 2, 2);
    Serial.println("Send signal");
    counter--;
  }
  Serial.println("Start pause");
  delay(50000); // pause, abnormally large, since this is another type of base.
}