#include <Arduino.h>
#define IR_SEND_PIN A2
#include <IRremote.h>
#include <IRremoteInt.h>


// fence readers
int FencereaderPinLeft = A6;
int FencereaderPinRight = A7;

// Light Indicators
int GreenLedPin = 7;
int RedLedPin = 6;

IRsend irsend = IRsend();

void setup()
{  
  Serial.begin(115200);
  // pinMode(IrTransmiterPin, OUTPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);
 digitalWrite(RedLedPin, LOW);
 digitalWrite(GreenLedPin, LOW);
}


boolean ReadFenceSensors(){
  int left = analogRead(FencereaderPinLeft);
  if(left < 100)
  {
    Serial.println(left);
    return true;
  }
  int right = analogRead(FencereaderPinRight);
  if (right < 100)
  {
    Serial.println(right);
    return true;
  }
  return false;
}


int status = 0;

void loop(){
    if(ReadFenceSensors())
    {
        // this allows about 6 to 10 seconds to recover in case signal is lost.
        status = status + 10;
        if(status>10){
          digitalWrite(GreenLedPin, HIGH);
          digitalWrite(RedLedPin, LOW);
          // tells the robot that fence is enabled.
          irsend.sendSony(1, 2, 2); // 410 == 1 and 400 == 0
          Serial.println(status);
          if (status> 100)
          {
            status=100;
          }
        }
    }
    else {
        status--;
        if(status<0){
          digitalWrite(GreenLedPin, LOW);
          digitalWrite(RedLedPin, HIGH);
          Serial.println(status);
          status=0;
        }
        // tells the robot that the fence went offline, so it is asking it to come back.
        irsend.sendSony(0, 2, 2); // 410 == 1 and 400 == 0
    }
}