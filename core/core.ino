// #include <U8x8lib.h>
#include <Arduino.h>

// screen
//U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

// fence readers
int FencereaderPinLeft = A6;
int FencereaderPinRight = A7;

// IR sensor
int IrSensorPin = A5;

// bumper buttons
int BumperButtonLeft = A3;
int BumperButtonRight = A1;
volatile byte StateBumperLeft = HIGH;
volatile byte StateBumperRight = HIGH;

// ultrasonic sensors
String DistancesDebug = "---------------";
int sensorLeft =11;
int sensorCenter = 12;
int sensorRight = 7;

// motors
int GrassCutterPin = A2;
int DirectionRight = 1;
int DirectionLeft = 2;
int RecommendedDirection = 0;
int MotorSpeeds = 255;
int MotorPinSpeedRight = 9;
int MotorPinSpeedLeft = 10;

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
  pinMode(MotorPinSpeedRight, OUTPUT);
  pinMode(MotorPinSpeedLeft, OUTPUT);
  pinMode(MotorPinLeft1, OUTPUT);
  pinMode(MotorPinLeft2, OUTPUT);
  pinMode(MotorPinRight1, OUTPUT);
  pinMode(MotorPinRight2, OUTPUT);
  pinMode(GrassCutterPin, OUTPUT);
  analogWrite(MotorPinSpeedRight, MotorSpeeds);
  analogWrite(MotorPinSpeedLeft, MotorSpeeds);
}

void SetStopWheels(){
    PrintScreen("Stop Motors.");
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
  PrintScreen("Move Left");
  SetMoveLeftMotor(1);
  SetMoveRightMotor(-1);
}
void SetMoveRight(){
  PrintScreen("Move right");
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(1);
}

void SetMoveFront(){
  PrintScreen("Move Front");
  SetMoveLeftMotor(1);
  SetMoveRightMotor(1);
}
void SetMoveBack(){
  PrintScreen("Move Back");
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
  int distanceLeft = ReadSingleUltrasonicSensor(sensorLeft);
  int distanceCenter = ReadSingleUltrasonicSensor(sensorCenter);
  int distanceRight = ReadSingleUltrasonicSensor(sensorRight);
  // Prints the distance on the Serial Monitor
  DistancesDebug = DistancesDebug + distanceLeft+"/";
  DistancesDebug = DistancesDebug + distanceCenter +"/";
  DistancesDebug = DistancesDebug + distanceRight;
  boolean collisionDetected = min(distanceLeft, min(distanceCenter, distanceRight)) < 30;
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

void ReadIrSensor(){
  // IrSensorPin
  int analogIr = analogRead(IrSensorPin);
  int digitalIr = digitalRead(IrSensorPin);
  Serial.println("IR analogIr ");
  Serial.println(analogIr);
  Serial.println("IR digitalIr ");
  Serial.println(digitalIr);
  delay(50);
}

boolean ReadFenceSensors(){
  int left = analogRead(FencereaderPinLeft);
  if(left < 75)
  {
    RecommendedDirection = DirectionRight;
    // PrintScreen("FenceLeft "+left);
    return true;
  }
  int right = analogRead(FencereaderPinRight);
  if (right < 75)
  {
    RecommendedDirection = DirectionLeft;
    // PrintScreen("FenceRight " +right);
    return true;
  }
  return false;
}


// Setup robot microcontroller initial state.
void setup()
{  
  Serial.begin(115200);
  delay(5000);
  SetUpMotorPins();
  // Delays the start to give time to the user to walk back.
  //u8x8.begin();
  //u8x8.clear();
  //u8x8.setFont(u8x8_font_victoriabold8_r);
  //u8x8.setCursor(0,0);
  PrintScreen("New Core v4!");
  //u8x8.println(F("Starting..."));
  CurrentSubMode = SUB_MODE_NAVIGATING;
  CurrentMode = MODE_MOWING;
  // bumper setup
  pinMode(BumperButtonLeft, INPUT_PULLUP);
  pinMode(BumperButtonRight, INPUT_PULLUP);

  pinMode(IrSensorPin, INPUT);

  Navigate();
}

bool ReadBumperSensors(){
  StateBumperLeft = digitalRead(BumperButtonLeft);
  StateBumperRight = digitalRead(BumperButtonRight);
  if(StateBumperLeft ==  LOW){
    RecommendedDirection = DirectionRight;
    return true;
  }
  if(StateBumperRight ==  LOW){
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

void UpdateScreenStats()
{
  String bat = "";
  bat =  bat + "Bat:"+CurrentBatteryPercentage;
  bat =  bat+"%@"+ CurrentVoltage + "v";
  //bat = bat+"V";
  String oldDebug = CurrentBatteryPercentage + CurrentDebug1 + CurrentDebug2 + CurrentDebug3;
  if(CurrentScreen == oldDebug)
  {
    return;
  }
  //battery: (%/volts)
  //u8x8.clear();
  //u8x8.setCursor(0,0);
  //u8x8.println(bat);
  //u8x8.println(DistancesDebug);
  //u8x8.println(CurrentDebug1);
  //u8x8.println(CurrentDebug2);
  //u8x8.println(CurrentDebug3);
  //u8x8.println(CurrentDebug4);
  //u8x8.println(CurrentDebug5);
  CurrentScreen = oldDebug;
}

void PrintScreen(String message){
  if(message == CurrentDebug1){
    // save screen
    return;
  }
  CurrentDebug5 = CurrentDebug4;
  CurrentDebug4 = CurrentDebug3;
  CurrentDebug3 = CurrentDebug2;
  CurrentDebug2 = CurrentDebug1;
  CurrentDebug1 = message;
}

void NavigateAndAvoidObstacles(String objective){
    if (CurrentSubMode == SUB_MODE_REDIRECTING)
    {
      StopCuttingGrass();
      Redirect();
    }else if( CurrentSubMode == SUB_MODE_NAVIGATING)
    {
      // PrintScreen(objective);
      Navigate();
    }
}

void Navigate(){
    // Check sensors
    if(GetSensorState())
    {
      // something happens? relocate
      CurrentSubMode = SUB_MODE_REDIRECTING;
      // PrintScreen("REDIRECTING");
    }else{
      // PrintScreen("NAVIGATING");
    }
    // nothing happens continue navigating
}

void Redirect()
{
    // Move back enough to not hit anything
    PrintScreen("Collision");
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
    // PrintScreen("NAVIGATING");
}

void SetModeByBatteryPercentage(){
  if(CurrentBatteryPercentage>=60)
  {
    // ask if can mow
    if(CurrentMode == MODE_CHARGING || CurrentMode == MODE_RETURNING_HOME){
      CurrentMode = MODE_MOWING;
      CurrentSubMode = SUB_MODE_NAVIGATING;
      SetMoveFront();
      PrintScreen("Battery Ready.");
    }
  }
  else if(CurrentBatteryPercentage<10){ // Stop moving if battery is lower than x%
    CurrentMode = MODE_CHARGING;
    PrintScreen("Low Battery.");
    SetStopWheels();
    PrintScreen("RECHARGING");
  }
  else if(CurrentBatteryPercentage<40)
  {
    // look for charging
    if(CurrentMode == MODE_MOWING)
    {
      CurrentMode = MODE_RETURNING_HOME;
      CurrentSubMode = SUB_MODE_NAVIGATING;
    }
  }
}

void StopCuttingGrass(){  
  digitalWrite(GrassCutterPin, LOW);
}
void CutGrass(){
  digitalWrite(GrassCutterPin, HIGH);
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
    // SetStopWheels();
    // can it dock?
    // if yes, start docking proccess
    // else
    // RedirectToHome();
    delay(500);
  }
  else if(CurrentMode ==  MODE_RESTING)
  {
    PrintScreen("RESTING");
    SetStopWheels();
  }
  else if(CurrentMode ==  MODE_CHARGING)
  {
    SetStopWheels();
    PrintScreen("RECHARGING");
  }
  else{
    PrintScreen("Err:"+CurrentMode);
    SetStopWheels();
  }
  GetBatteryVoltage();
  SetModeByBatteryPercentage();
  GetSensorState();
  UpdateScreenStats();
  ReadIrSensor();
}
