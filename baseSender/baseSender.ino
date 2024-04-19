
#include <Arduino.h>

int IrTransmiterPin = A2;

void setup()
{  
  Serial.begin(115200);
  pinMode(IrTransmiterPin, OUTPUT);

}

void loop(){
    analogWrite(IrTransmiterPin, 100);
}