// #include <U8x8lib.h>
#include <Arduino.h>

// screen
//U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
#include <IRremote.h>
#include <IRremoteInt.h>
IRrecv irsend = IRrecv();

/*
Pins used:
A1: BumperButtonRight
A2: GrassCutterPin
A3: BumperButtonLeft
A4: IrSensorPinLeft
A5: IrSensorPinRight
A6: FencereaderPinLeft
A7: FencereaderPinRight
D2: MotorPinRight2
D3: MotorPinRight1
D4: FREE
D5: MotorPinLeft2
D6: MotorPinLeft1
D7: UltrasonicSensorRight
D8: FREE
D9: BlueLedPin
D10: RedLedPin
D11: UltrasonicSensorLeft
D12: UltrasonicSensorCenter
D13: FREE

*/


// Light Indicators
int BlueLedPin = 9;
int RedLedPin = 10;


// fence readers
int FencereaderPinLeft = A6;
int FencereaderPinRight = A7;

// IR sensor
int IrSensorPinRight = A4;
int IrSensorPinLeft = A5;
int RecommendedDirectionIr = 0;


// bumper buttons
int BumperButtonLeft = A3;
int BumperButtonRight = A1;
volatile byte StateBumperLeft = HIGH;
volatile byte StateBumperRight = HIGH;

// ultrasonic sensors
String DistancesDebug = "---------------";
int UltrasonicSensorLeft =11;
int UltrasonicSensorCenter = 12;
int UltrasonicSensorRight = 7;

// motors
int GrassCutterPin = A2;
int DirectionNone = 1;
int DirectionRight = 1;
int DirectionLeft = 2;
int DirectionCenter = 3;
int RecommendedDirection = 0;
int MotorSpeeds = 255;

 int MotorPinRight1 = 3;
 int MotorPinRight2 = 2;

 int MotorPinLeft1 = 6;
 int MotorPinLeft2 = 5;


// Mode:
String CurrentDebug1 = "Starting...";
String CurrentDebug2 = "";
String CurrentDebug3 = "";
String CurrentDebug4 = "";
String CurrentDebug5 = "";
String CurrentScreen = "1";

int MODE_CHARGING = -2;
int MODE_RESTING = -1;
int MODE_MOWING = 0;
int MODE_RETURNING_HOME = 2;

int SUB_MODE_NAVIGATING = 0; // moving fordward
int SUB_MODE_REDIRECTING = 1; // avoiding obstacles

int CurrentMode = MODE_MOWING;
int CurrentSubMode = 0;

// batteries:
int VoltageReaderPin = A0;
float CurrentVoltage = 12.6;
float MarginVoltage = 0.05;
int CurrentBatteryPercentage = 0;

/*
  MOTORS
*/
void SetUpMotorPins(){
  pinMode(MotorPinLeft1, OUTPUT);
  pinMode(MotorPinLeft2, OUTPUT);
  pinMode(MotorPinRight1, OUTPUT);
  pinMode(MotorPinRight2, OUTPUT);
  pinMode(GrassCutterPin, OUTPUT);
}

void SetStopWheels(){
    StopCuttingGrass();
    analogWrite(MotorPinLeft1, LOW);
    analogWrite(MotorPinLeft2, LOW);
    analogWrite(MotorPinRight1, LOW);
    analogWrite(MotorPinRight2, LOW);
    delay(3000);
}

void SetMoveLeftMotor(int direction){
  if(direction==1){
    analogWrite(MotorPinLeft1, MotorSpeeds);
    analogWrite(MotorPinLeft2, LOW);
  }else{
    analogWrite(MotorPinLeft1, LOW);
    analogWrite(MotorPinLeft2, MotorSpeeds);
  }
}
void SetMoveRightMotor(int direction){
  if(direction==1){
    analogWrite(MotorPinRight1, MotorSpeeds);
    analogWrite(MotorPinRight2, LOW);
  }else{
    analogWrite(MotorPinRight1, LOW);
    analogWrite(MotorPinRight2, MotorSpeeds);
  }
}

void SetMoveLeft(){
  SetMoveLeftMotor(1);
  SetMoveRightMotor(-1);
}
void SetMoveRight(){
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(1);
}

void SetMoveFront(){
  SetMoveLeftMotor(1);
  SetMoveRightMotor(1);
}
void SetMoveBack(){
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(-1);
}
/*
  END MOTORS
*/

int ReadSingleUltrasonicSensor(int sensor){
  delayMicroseconds(100);
  pinMode(sensor, OUTPUT);
  digitalWrite(sensor, LOW);
  digitalWrite(sensor, HIGH);
  delayMicroseconds(100);
  digitalWrite(sensor, LOW);
  pinMode(sensor, INPUT);
  return pulseIn(sensor, HIGH)* 0.034 /2;
}

boolean ReadUltrasonicSensor(){
  DistancesDebug = "Dis:";
  int distanceLeft = ReadSingleUltrasonicSensor(UltrasonicSensorLeft);
  int distanceCenter = ReadSingleUltrasonicSensor(UltrasonicSensorCenter);
  int distanceRight = ReadSingleUltrasonicSensor(UltrasonicSensorRight);
  // Prints the distance on the Serial Monitor
  DistancesDebug = DistancesDebug + distanceLeft+"/";
  DistancesDebug = DistancesDebug + distanceCenter +"/";
  DistancesDebug = DistancesDebug + distanceRight;

  int distanceAllowed = 30;
  if(CurrentMode == MODE_RETURNING_HOME)
  {
    distanceAllowed =  10;
  }
  boolean collisionDetected = min(distanceLeft, min(distanceCenter, distanceRight)) < distanceAllowed;
  if(collisionDetected)
  {
    if(distanceLeft < distanceRight)
    {
      RecommendedDirection = DirectionRight;
    }else{
      RecommendedDirection = DirectionLeft;
    }
  }
  return collisionDetected;
}

bool ReadIrSensor(){
  int analogIrLeft = analogRead(IrSensorPinLeft);
  int analogIrRight = analogRead(IrSensorPinRight);
  int readings = 100;
  bool detected = false;
  RecommendedDirectionIr = DirectionNone;
  int detectedLeft = 0;
  int detectedRight = 0;
  // does multiple readings to make sure both sensors capture data
  // TODO: replace by interrupts, it will remove the need of loop
  while(readings)
  {
    readings--;
    if(analogIrLeft < 1000) {
      detected = true;
      detectedLeft++;
    }
    if(analogIrRight < 1000) {
      Serial.println(analogIrRight);
      detectedRight++;
      detected = true;
    }
    RecommendedDirectionIr = 0;
    analogIrLeft = analogRead(IrSensorPinLeft);
    analogIrRight = analogRead(IrSensorPinRight);
  }
  if(detectedLeft>0 && detectedRight>0) {
    // both sensors detect the base signal, so recommends to move fordwardish.
    RecommendedDirection = DirectionCenter;
  } else if (detectedRight){
    RecommendedDirectionIr = DirectionRight;
  } else if (detectedLeft){
    RecommendedDirectionIr = DirectionLeft;
  }
  return detected;
}

boolean ReadFenceSensors(){
  int left = analogRead(FencereaderPinLeft);
  if(left < 100)
  {
    RecommendedDirection = DirectionRight;
    SetStopWheels();  // TODO: remove
    BlinkLedPin(BlueLedPin, 5); // TODO: remove
    SetMoveFront(); // TODo: remove
    return true;
  }
  int right = analogRead(FencereaderPinRight);
  if (right < 100)
  {
    RecommendedDirection = DirectionLeft;
    SetStopWheels();  // TODO: remove
    BlinkLedPin(BlueLedPin, 5); // TODO: remove
    SetMoveFront(); // TODo: remove
    return true;
  }
  return false;
}


// Setup robot microcontroller initial state.
void setup()
{  
  Serial.begin(115200);
  SetUpMotorPins();
  CurrentSubMode = SUB_MODE_NAVIGATING;
  CurrentMode = MODE_MOWING;
  // bumper setup
  pinMode(BumperButtonLeft, INPUT_PULLUP);
  pinMode(BumperButtonRight, INPUT_PULLUP);

  //pinMode(IrSensorPin, INPUT);
  pinMode(IrSensorPinLeft, INPUT);
  pinMode(IrSensorPinRight, INPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(BlueLedPin, OUTPUT);
  Navigate();
  BlinkLedPin(BlueLedPin, 5);
  BlinkLedPin(RedLedPin, 5);
  digitalWrite(BlueLedPin, HIGH);
  digitalWrite(RedLedPin, HIGH);

}

bool ReadBumperSensors(){
  StateBumperLeft = digitalRead(BumperButtonLeft);
  StateBumperRight = digitalRead(BumperButtonRight);
  if(StateBumperLeft == LOW){
    RecommendedDirection = DirectionRight;
    return true;
  }
  if(StateBumperRight == LOW){
    RecommendedDirection = DirectionLeft;
    return true;
  }
  return false;
}

bool GetSensorState(){
  /*
    This is a basic alert from the sensors, if true, sensors are asking to turn arround.
    If false, sensors do not find anything.
  */
  return ReadBumperSensors() || ReadUltrasonicSensor() || ReadFenceSensors();
}

int GetBatteryPercentage()
{
  if(CurrentVoltage>12.59)
    return 100;
  if(CurrentVoltage>12.44)
    return 95;
  if(CurrentVoltage>12.32)
    return 90;
  if(CurrentVoltage>12.23)
    return 85;
  if(CurrentVoltage>12.05)
    return 80;
  if(CurrentVoltage>11.93)
    return 75;
  if(CurrentVoltage>11.84)
    return 70;
  if(CurrentVoltage>11.72)
    return 65;
  if(CurrentVoltage>11.61)
    return 60;
  if(CurrentVoltage>11.55)
    return 55;
  if(CurrentVoltage>11.52)
    return 50;
  if(CurrentVoltage>11.46)
    return 45;
  if(CurrentVoltage>11.40)
    return 40;
  if(CurrentVoltage>11.37)
    return 35;
  if(CurrentVoltage>11.31)
    return 30;
  if(CurrentVoltage>11.25)
    return 25;
  if(CurrentVoltage>11.19)
    return 20;
  if(CurrentVoltage>11.13)
    return 15;
  if(CurrentVoltage>11.07)
    return 10;
  return 0;
}

void GetBatteryVoltage(){
  // 12.6 = 100
  // v = X
  float R1 = 47000.0;
  float R2 = 10000.0;
  float voltage = analogRead(VoltageReaderPin) * (5.0/1024)*((R1 + R2)/R2);
  CurrentVoltage = ((CurrentVoltage + voltage)/2) + MarginVoltage;
  CurrentBatteryPercentage = GetBatteryPercentage();
}


void NavigateAndAvoidObstacles(String objective){
    if (CurrentSubMode == SUB_MODE_REDIRECTING)
    {
      StopCuttingGrass();
      Redirect();
    }else if( CurrentSubMode == SUB_MODE_NAVIGATING)
    {
      Navigate();
    }
}

void Navigate(){
    // Check sensors
    if(GetSensorState())
    {
      CurrentSubMode = SUB_MODE_REDIRECTING;
    }
}

void Redirect()
{
    // Collision or redirection
    // Move back enough to not hit anything
    // turn arround
    SetMoveBack();
    delay(random(500, 3000)); // time depends of the RPM of the motors
    if(RecommendedDirection == DirectionLeft)
    {
      SetMoveLeft();
    }else{
      SetMoveRight();
    }
    delay(random(500, 3000)); //move to the side from 0.5 to 5 seconds
    SetMoveFront();
    CutGrass();
    // continue for next cicle
    CurrentSubMode = SUB_MODE_NAVIGATING;
}

void SetModeByBatteryPercentage(){
  if(CurrentBatteryPercentage>=95)
  {
    // ask if can mow
    if(CurrentMode == MODE_CHARGING || CurrentMode == MODE_RETURNING_HOME) {
      CurrentMode = MODE_MOWING;
      CurrentSubMode = SUB_MODE_NAVIGATING;
      SetMoveFront();
      digitalWrite(BlueLedPin, HIGH);
      digitalWrite(RedLedPin, LOW);
    }
  }
  else if(CurrentBatteryPercentage<10) { // Stop moving if battery is lower than x%
    CurrentMode = MODE_CHARGING;
    SetStopWheels();
    digitalWrite(BlueLedPin, LOW);
    digitalWrite(RedLedPin, HIGH);
  }
  else if(CurrentBatteryPercentage<40) // TODO: revert to 40
  {
    // look for charging
    if(CurrentMode == MODE_MOWING)
    {
      //digitalWrite(BlueLedPin, LOW);
      BlinkLedPin(BlueLedPin, 2);
      CurrentMode = MODE_RETURNING_HOME;
      CurrentSubMode = SUB_MODE_NAVIGATING;
      Serial.println("RETURNING HOME");
    }
  }
}

void StopCuttingGrass(){  
  digitalWrite(GrassCutterPin, LOW);
}
void CutGrass(){
  digitalWrite(GrassCutterPin, HIGH);
}

void BlinkLedPin(int LedPinNumber, int Times){
  int originalState = digitalRead(LedPinNumber);
  while(Times>0) {
    digitalWrite(LedPinNumber, HIGH);
    delay(200);
    digitalWrite(LedPinNumber, LOW);
    delay(200);
    Times--;
  }
  // return to original state
  digitalWrite(LedPinNumber, originalState);
}

void loop()
{
  if(CurrentMode == MODE_MOWING) {
     NavigateAndAvoidObstacles("MOWING");
  }
  else if(CurrentMode ==  MODE_RETURNING_HOME)
  {
    NavigateAndAvoidObstacles("RETURNING HOME");
    // Is home detected?
    if(ReadIrSensor())
    {
      Serial.println("HOME DETECTED");
      // it detects the base:
      if(RecommendedDirectionIr == DirectionLeft)
      {
        SetMoveLeft();
        delay(random(500, 1500));
        Serial.println("Move a bit to the left.");
        SetMoveFront();
      }
      else if (RecommendedDirectionIr == DirectionRight)
      {
        SetMoveRight();
        delay(random(500, 1500));
        Serial.println("Move a bit to the right.");
        SetMoveFront();
      }
      else if (RecommendedDirectionIr == DirectionCenter)
      {
        // it seems both sensors are looking the target
        if(ReadUltrasonicSensor()) // it is 10cm at the base
        {
          // Stop moving
          CurrentMode = MODE_CHARGING;
          SetStopWheels();
          RecommendedDirectionIr = DirectionNone;
          Serial.println("Parcked for charging!");
          BlinkLedPin(BlueLedPin, 5);
        }
      }
    }
  }
  else if(CurrentMode ==  MODE_RESTING)
  {
    SetStopWheels();
  }
  else if(CurrentMode ==  MODE_CHARGING)
  {
    SetStopWheels();
    BlinkLedPin(RedLedPin, 3);
    BlinkLedPin(BlueLedPin, CurrentBatteryPercentage/10);
  }
  else{
    SetStopWheels();
    /// Unknown error/state flash all leds:
    digitalWrite(RedLedPin, HIGH);
    digitalWrite(BlueLedPin, HIGH);
    delay(2000);
    digitalWrite(RedLedPin, LOW);
    digitalWrite(BlueLedPin, LOW);
  }
  GetBatteryVoltage();
  SetModeByBatteryPercentage();
  GetSensorState();
}
