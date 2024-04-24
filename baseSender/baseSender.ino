// #include <Arduino.h>
#define IR_SEND_PIN A2
#include <IRremote.h>
#include <IRremoteInt.h>

// int IrTransmiterPin = A2;
int iPinPulse = 3;
IRsend irsend = IRsend();

void setup()
{  
  Serial.begin(115200);
  // pinMode(IrTransmiterPin, OUTPUT);
  pinMode(iPinPulse, OUTPUT);

}

int counter = 0;

void loop(){
  // Create a 5KHz pulse for the digital fence
  // Set digital fence high
  analogWrite(iPinPulse, 255);
  delayMicroseconds(100);
  // Set digital fence low
  analogWrite(iPinPulse, 0);
  delayMicroseconds(100);
  // IR sender, just send a code
  irsend.sendSony(2, 2, 2);
}